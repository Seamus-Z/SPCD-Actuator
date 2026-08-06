#!/usr/bin/env python3
"""CAN-FD client and raw-binary flasher for the xtellar bootloader.

Examples:
  tools/bootload_test.py --enter
  tools/bootload_test.py --flash bazel-bin/fw/app.bin

The input to --flash must be a raw application binary linked at 0x08010000,
not the combined bootloader+application image.
"""

import argparse
import sys
import time


HOST_ID = 0
BOOT_ID = 1
APP_START = 0x08010000
BOOT_REQUEST_ID = 0x7E
BOOT_REQUEST_PAYLOAD = b"BOOT"

FLASH_PAGE_SIZE = 2048
FAST_WRITE_SIZE = 24
FAST_WRITE_SETTLE_DELAY_S = 0.002
FAST_PAGE_ERASE_SETTLE_DELAY_S = 0.05
SAFE_WRITE_SIZE = 8
SAFE_WRITE_SETTLE_DELAY_S = 0.05
SAFE_PAGE_ERASE_SETTLE_DELAY_S = 0.25
WRITE_TIMEOUT_S = 10.0
RESPONSE_POLL_INTERVAL_S = 0.5
PAGE_WRITE_ATTEMPTS = 5
VERIFY_ATTEMPTS = 5
POWER_RECOVERY_DELAY_S = 2.0


def varuint_encode(value):
    result = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        result.append(byte | (0x80 if value else 0))
        if not value:
            return bytes(result)


def varuint_decode(data, offset):
    result = 0
    shift = 0
    for _ in range(5):
        if offset >= len(data):
            return None, offset
        byte = data[offset]
        offset += 1
        result |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return result, offset
        shift += 7
    return None, offset


def build_write(data):
    return (varuint_encode(0x40) + varuint_encode(1) +
            varuint_encode(len(data)) + data)


def build_poll(max_bytes=61):
    return (varuint_encode(0x42) + varuint_encode(1) +
            varuint_encode(max_bytes))


def parse_response(data):
    subframe, offset = varuint_decode(data, 0)
    channel, offset = varuint_decode(data, offset)
    length, offset = varuint_decode(data, offset)
    if subframe != 0x41 or channel != 1 or length is None:
        raise RuntimeError(f"malformed multiplex response: {bytes(data).hex()}")
    if offset + length > len(data):
        raise RuntimeError("truncated multiplex response")
    return bytes(data[offset:offset + length])


class BootloaderClient:
    def __init__(self, bus, can_module, verbose=False, safe_writes=False):
        self.bus = bus
        self.can = can_module
        self.verbose = verbose
        self.safe_writes = safe_writes
        self.response_id = (BOOT_ID << 8) | HOST_ID
    def _send(self, arbitration_id, data, extended, fd=True):
        self.bus.send(self.can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=extended,
            is_fd=fd,
            # Keep all multiplex traffic at the nominal rate initially.  It
            # remains CAN-FD (for 64-byte commands), but does not depend on a
            # separately matched data-phase timing configuration.
            bitrate_switch=False,
            data=data))

    def enter(self):
        # Spam BOOT while draining RX. APP may flood Tel+Enc (~1 kHz); gs_usb
        # then reports ENOBUFS on TX and a single BOOT never leaves the host.
        deadline = time.monotonic() + 5.0
        sent = 0
        while time.monotonic() < deadline:
            # Free USB adapter buffers first.
            while self.bus.recv(0) is not None:
                pass
            try:
                self._send(BOOT_REQUEST_ID, BOOT_REQUEST_PAYLOAD, False, fd=False)
                sent += 1
            except Exception:
                pass
            time.sleep(0.02)
        if self.verbose:
            print(f"BOOT sent {sent} times; waiting for bootloader startup")
        # APP blinks 3x then resets; BL blinks before FDCAN init (HSI ~16 MHz).
        drain_until = time.monotonic() + 2.0
        while time.monotonic() < drain_until:
            while self.bus.recv(0) is not None:
                pass
            time.sleep(0.05)

    def _receive_response(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.bus.recv(timeout=min(0.1, deadline - time.monotonic()))
            if frame is None or frame.arbitration_id != self.response_id:
                continue
            return parse_response(frame.data)
        raise TimeoutError("bootloader did not answer a multiplex poll")

    def poll(self, timeout=1.0, retries=1):
        query_id = ((HOST_ID | 0x80) << 8) | BOOT_ID
        last_error = None
        for _ in range(retries):
            self._send(query_id, build_poll(), True, fd=False)
            try:
                return self._receive_response(timeout)
            except TimeoutError as error:
                last_error = error
                time.sleep(0.1)
        raise last_error

    def drain_banner(self):
        result = bytearray()
        for _ in range(8):
            # The first query after an APP->BL reset may race the end of the
            # bootloader startup indication.  Retrying a read-only poll is
            # safe and avoids requiring exact LED-loop timing.
            chunk = self.poll(retries=5)
            if not chunk:
                break
            result += chunk
        if result and self.verbose:
            print(result.decode("ascii", errors="replace").rstrip())
        return bytes(result)

    def command(self, text, expect_response=True, timeout=3.0):
        # Commands are writes, not queries.  The result is retrieved by a
        # subsequent poll, matching moteus.Stream's diagnostic semantics.
        payload = text.encode("ascii") + b"\n"
        if len(build_write(payload)) > 64:
            raise ValueError(f"command does not fit one CAN-FD frame: {text!r}")
        write_id = (HOST_ID << 8) | BOOT_ID
        self._send(write_id, build_write(payload), True)
        if self.verbose:
            print(f"> {text}")
        if not expect_response:
            return b""

        deadline = time.monotonic() + timeout
        result = bytearray()
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            try:
                # A poll sent while Flash is busy can be lost by the target.
                # Re-send read-only polls instead of waiting the full write
                # timeout on one query.
                chunk = self.poll(
                    timeout=min(RESPONSE_POLL_INTERVAL_S, remaining))
            except TimeoutError:
                continue
            result += chunk
            if b"\n" in result:
                line = bytes(result).strip()
                if self.verbose:
                    print(f"< {line.decode('ascii', errors='replace')}")
                if line.startswith(b"ERR"):
                    raise RuntimeError(line.decode("ascii", errors="replace"))
                return line
            time.sleep(0.002)
        raise TimeoutError(f"no complete response to {text!r}")

    def _recover_flash_session(self):
        """Wait for a brownout reset, then restore an unlocked BL session."""
        last_error = None
        for attempt in range(1, PAGE_WRITE_ATTEMPTS + 1):
            time.sleep(POWER_RECOVERY_DELAY_S)
            try:
                try:
                    # A reset while page 0 is erased leaves the APP invalid,
                    # so the target normally remains in the bootloader.
                    self.drain_banner()
                    response = self.command("echo ready", timeout=3.0)
                except (TimeoutError, RuntimeError):
                    # A reset on a later page can boot the still-valid APP.
                    # Re-send the one-shot BOOT request and return to BL.
                    self.enter()
                    self.drain_banner()
                    response = self.command("echo ready", timeout=5.0)
                if response != b"ready":
                    raise RuntimeError(
                        f"unexpected bootloader echo: {response!r}")
                self.command("unlock", timeout=5.0)
                return
            except (TimeoutError, RuntimeError) as error:
                last_error = error
                if self.verbose:
                    print(
                        f"bootloader recovery {attempt}/"
                        f"{PAGE_WRITE_ATTEMPTS} failed: {error}")
        raise RuntimeError(
            "bootloader did not recover after a write failure") from last_error

    def _write_page(self, page_offset, page, image_size):
        """Write one erase page; retry the whole page after any interruption."""
        for attempt in range(1, PAGE_WRITE_ATTEMPTS + 1):
            if attempt > 1:
                self._recover_flash_session()
                if self.verbose:
                    print(
                        f"retrying flash page 0x{APP_START + page_offset:08x} "
                        f"({attempt}/{PAGE_WRITE_ATTEMPTS})")
            if self.safe_writes:
                write_size = SAFE_WRITE_SIZE
                write_delay = SAFE_WRITE_SETTLE_DELAY_S
                erase_delay = SAFE_PAGE_ERASE_SETTLE_DELAY_S
            else:
                write_size = FAST_WRITE_SIZE
                write_delay = FAST_WRITE_SETTLE_DELAY_S
                erase_delay = FAST_PAGE_ERASE_SETTLE_DELAY_S
            try:
                for block_offset in range(0, len(page), write_size):
                    block = page[block_offset:block_offset + write_size]
                    address = APP_START + page_offset + block_offset
                    self.command(
                        f"w {address:08x} {block.hex()}",
                        timeout=WRITE_TIMEOUT_S)
                    # The command reply means Flash is no longer busy.  The
                    # remaining delay only gives a marginal supply time to
                    # recover from the programming-current pulse.
                    delay = erase_delay if block_offset == 0 else write_delay
                    time.sleep(delay)
                    completed = min(
                        page_offset + block_offset + len(block), image_size)
                    self._progress("write", completed, image_size)
                return
            except (TimeoutError, RuntimeError) as error:
                if not self.safe_writes:
                    self.safe_writes = True
                    if self.verbose:
                        print(
                            "\nfast write failed; retrying this page and all "
                            "remaining pages with safe pacing")
                if self.verbose:
                    print(
                        f"\nflash page 0x{APP_START + page_offset:08x} "
                        f"attempt {attempt}/{PAGE_WRITE_ATTEMPTS} failed: "
                        f"{error}")
                if attempt == PAGE_WRITE_ATTEMPTS:
                    raise RuntimeError(
                        f"failed to program flash page at "
                        f"0x{APP_START + page_offset:08x}; repeated failure "
                        "usually means the target supply is still sagging "
                        "during Flash programming") from error

    def _read_verified_block(self, address, expected):
        last_error = None
        for attempt in range(1, VERIFY_ATTEMPTS + 1):
            try:
                response = self.command(
                    f"r {address:08x} {len(expected):x}", timeout=5.0)
                fields = response.split()
                if len(fields) != 2 or int(fields[0], 16) != address:
                    raise RuntimeError(
                        f"malformed read response at 0x{address:08x}")
                actual = bytes.fromhex(fields[1].decode("ascii"))
                if len(actual) != len(expected):
                    raise RuntimeError(
                        f"truncated read response at 0x{address:08x}")
                if actual != expected:
                    raise RuntimeError(
                        f"verify failed at 0x{address:08x}")
                return
            except (TimeoutError, RuntimeError, ValueError) as error:
                last_error = error
                if attempt < VERIFY_ATTEMPTS:
                    if self.verbose:
                        print(
                            f"\nverify retry {attempt}/"
                            f"{VERIFY_ATTEMPTS} at 0x{address:08x}: {error}")
                    time.sleep(0.1)
        raise RuntimeError(
            f"unable to verify flash at 0x{address:08x}") from last_error

    def flash(self, image):
        if not image:
            raise ValueError("application image is empty")
        if len(image) > 0x70000:
            raise ValueError("application image exceeds 448 KiB")

        # Fully populate the final flash double-word so a brownout before
        # "lock" cannot lose bytes buffered only in the bootloader's RAM.
        padded_image = image + b"\xff" * ((-len(image)) % 8)

        if self.verbose:
            mode = "safe 8-byte" if self.safe_writes else "fast 24-byte"
            print(f"Flash write mode: {mode}")
        self.command("unlock")
        for page_offset in range(0, len(padded_image), FLASH_PAGE_SIZE):
            page = padded_image[page_offset:page_offset + FLASH_PAGE_SIZE]
            self._write_page(page_offset, page, len(image))
        self.command("lock", timeout=WRITE_TIMEOUT_S)
        print()

        read_size = 32
        for offset in range(0, len(image), read_size):
            expected = image[offset:offset + read_size]
            address = APP_START + offset
            self._read_verified_block(address, expected)
            self._progress("verify", offset + len(expected), len(image))
        print("\nFlash and verify complete; resetting into application")
        self.command("reset", expect_response=False)

    @staticmethod
    def _progress(action, done, total):
        print(f"\r{action:6s}: {done:7d}/{total:7d} bytes", end="", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--enter", action="store_true",
                        help="request bootloader mode using standard ID 0x7e/BOOT")
    parser.add_argument("--flash", metavar="APP.BIN",
                        help="enter BL, program a raw app binary, verify, reset")
    parser.add_argument(
        "--safe", action="store_true",
        help="force conservative 8-byte writes instead of adaptive fast mode")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    try:
        import can
    except ImportError:
        sys.exit("python-can is required: python3 -m pip install python-can")

    bus = can.Bus(interface="socketcan", channel=args.interface, fd=True)
    client = BootloaderClient(
        bus, can, verbose=args.verbose, safe_writes=args.safe)
    try:
        if args.enter or args.flash:
            client.enter()
        banner = client.drain_banner()
        # The banner is a one-shot stream message.  If the target was already
        # in bootloader mode, an earlier client/poll may have consumed it.  A
        # successfully parsed empty poll still proves that the bootloader is
        # alive; the echo command below is the authoritative protocol check.
        if not banner and args.verbose:
            print("(bootloader answered; startup banner was already consumed)")

        if args.flash:
            echo = client.command("echo ready")
            if b"ready" not in echo:
                raise RuntimeError(f"unexpected bootloader echo: {echo!r}")
            with open(args.flash, "rb") as source:
                client.flash(source.read())
        else:
            response = client.command("echo hello")
            if b"hello" not in response:
                raise RuntimeError(f"unexpected echo response: {response!r}")
            print("Bootloader echo test passed")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()

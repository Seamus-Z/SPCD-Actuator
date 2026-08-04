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
    def __init__(self, bus, can_module, verbose=False):
        self.bus = bus
        self.can = can_module
        self.verbose = verbose
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
            chunk = self.poll(timeout=max(0.1, deadline - time.monotonic()))
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

    def flash(self, image):
        if not image:
            raise ValueError("application image is empty")
        if len(image) > 0x70000:
            raise ValueError("application image exceeds 448 KiB")

        self.command("unlock")
        write_size = 24
        for offset in range(0, len(image), write_size):
            block = image[offset:offset + write_size]
            self.command(f"w {APP_START + offset:08x} {block.hex()}")
            self._progress("write", offset + len(block), len(image))
        self.command("lock")  # also flushes a final partial double-word
        print()

        read_size = 32
        for offset in range(0, len(image), read_size):
            expected = image[offset:offset + read_size]
            address = APP_START + offset
            response = self.command(f"r {address:08x} {len(expected):x}")
            fields = response.split()
            if len(fields) != 2 or int(fields[0], 16) != address:
                raise RuntimeError(f"malformed read response at 0x{address:08x}")
            actual = bytes.fromhex(fields[1].decode("ascii"))
            if actual != expected:
                raise RuntimeError(f"verify failed at 0x{address:08x}")
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
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    try:
        import can
    except ImportError:
        sys.exit("python-can is required: python3 -m pip install python-can")

    bus = can.Bus(interface="socketcan", channel=args.interface, fd=True)
    client = BootloaderClient(bus, can, args.verbose)
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

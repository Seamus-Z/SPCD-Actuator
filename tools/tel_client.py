#!/usr/bin/env python3
"""Query xtellar APP telemetry over the multiplex text tunnel.

Examples:
  tools/tel_client.py --list
  tools/tel_client.py --drv
  tools/tel_client.py --cmd status
  tools/tel_client.py --stream drv --hz 5
"""

import argparse
import sys
import time

HOST_ID = 0
DEVICE_ID = 1
TUNNEL_CHANNEL = 1


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
    return (varuint_encode(0x40) + varuint_encode(TUNNEL_CHANNEL) +
            varuint_encode(len(data)) + data)


def build_poll(max_bytes=61):
    return (varuint_encode(0x42) + varuint_encode(TUNNEL_CHANNEL) +
            varuint_encode(max_bytes))


def parse_response(data):
    subframe, offset = varuint_decode(data, 0)
    channel, offset = varuint_decode(data, offset)
    length, offset = varuint_decode(data, offset)
    if subframe != 0x41 or channel != TUNNEL_CHANNEL or length is None:
        raise RuntimeError(f"malformed multiplex response: {bytes(data).hex()}")
    if offset + length > len(data):
        raise RuntimeError("truncated multiplex response")
    return bytes(data[offset:offset + length])


class TelClient:
    def __init__(self, bus, can_module, verbose=False):
        self.bus = bus
        self.can = can_module
        self.verbose = verbose
        self.response_id = (DEVICE_ID << 8) | HOST_ID

    def _send(self, arbitration_id, data, extended=True, fd=True):
        self.bus.send(self.can.Message(
            arbitration_id=arbitration_id,
            is_extended_id=extended,
            is_fd=fd,
            bitrate_switch=False,
            data=data))

    def _receive(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.bus.recv(timeout=min(0.1, deadline - time.monotonic()))
            if frame is None or frame.arbitration_id != self.response_id:
                continue
            return parse_response(frame.data)
        raise TimeoutError("device did not answer a multiplex poll")

    def poll(self, timeout=1.0, retries=5):
        query_id = ((HOST_ID | 0x80) << 8) | DEVICE_ID
        last_error = None
        for _ in range(retries):
            self._send(query_id, build_poll(), True, fd=False)
            try:
                return self._receive(timeout)
            except TimeoutError as error:
                last_error = error
                time.sleep(0.05)
        raise last_error

    def drain(self):
        result = bytearray()
        for _ in range(8):
            chunk = self.poll()
            if not chunk:
                break
            result += chunk
        if result and self.verbose:
            print(result.decode("ascii", errors="replace").rstrip())
        return bytes(result)

    def command(self, text, timeout=2.0):
        payload = text.encode("ascii") + b"\n"
        write_id = (HOST_ID << 8) | DEVICE_ID
        self._send(write_id, build_write(payload), True, fd=True)
        if self.verbose:
            print(f"> {text}")

        deadline = time.monotonic() + timeout
        result = bytearray()
        while time.monotonic() < deadline:
            chunk = self.poll(timeout=max(0.1, deadline - time.monotonic()))
            result += chunk
            if b"\n" in result:
                line = bytes(result).strip()
                if self.verbose:
                    print(f"< {line.decode('ascii', errors='replace')}")
                return line
            time.sleep(0.002)
        raise TimeoutError(f"no complete response to {text!r}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--drv", action="store_true")
    parser.add_argument("--status", action="store_true")
    parser.add_argument("--cmd", metavar="TEXT", help="raw tunnel command")
    parser.add_argument("--stream", metavar="CMD", help="repeat a command")
    parser.add_argument("--hz", type=float, default=2.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    try:
        import can
    except ImportError:
        sys.exit("python-can is required: python3 -m pip install python-can")

    bus = can.Bus(interface="socketcan", channel=args.interface, fd=True)
    client = TelClient(bus, can, args.verbose)
    try:
        client.drain()

        if args.stream:
            period = 1.0 / args.hz if args.hz > 0 else 0.5
            while True:
                print(client.command(args.stream).decode("ascii", errors="replace"))
                time.sleep(period)

        if args.list:
            print(client.command("list").decode("ascii", errors="replace"))
        if args.drv:
            print(client.command("drv").decode("ascii", errors="replace"))
        if args.status:
            print(client.command("status").decode("ascii", errors="replace"))
        if args.cmd:
            print(client.command(args.cmd).decode("ascii", errors="replace"))

        if not (args.list or args.drv or args.status or args.cmd or args.stream):
            print(client.command("drv").decode("ascii", errors="replace"))
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()

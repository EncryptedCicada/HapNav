#!/usr/bin/env python3
"""
BLE receiver for the HapNav chest pin in laptop-link mode.

Pairs with firmware built with CONFIG_HAPNAV_LINK_LAPTOP=y
(pin/prj_laptop.conf). The pin advertises as "HapNav-Pin"; this script
finds it by service UUID (or name), connects, subscribes to the frame
characteristic, reassembles the chunked notifications into 219-byte
`struct hapnav_frame`s (layout: lib/ble/include/hapnav/ble_proto.h),
prints a periodic summary, and logs every frame as one JSON line to
Logs/ble_<timestamp>.jsonl at the project root.

Usage:
    pip install bleak
    python Tools/ble_receiver.py                 # auto-find, print every 10th frame
    python Tools/ble_receiver.py --print-every 1 # print every frame
    python Tools/ble_receiver.py --no-log

macOS: the terminal app needs Bluetooth permission
(System Settings > Privacy & Security > Bluetooth).
Auto-reconnects if the link drops. Ctrl+C to quit.
"""
from __future__ import annotations

import argparse
import asyncio
import datetime as dt
import json
import struct
import sys
import time
from pathlib import Path

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    sys.exit("bleak not installed: pip install bleak")

SVC_UUID = "5a9c0010-7e6f-4abc-9c12-aabbccdd1100"
FRAME_UUID = "5a9c0011-7e6f-4abc-9c12-aabbccdd1100"
DEFAULT_NAME = "HapNav-Pin"

# struct hapnav_frame (packed, little-endian):
#   u32 ts | f32 quat[4] | i16 dist[64] | u8 status[64] | u8 urg[4] | i16 near | u8 flags
FRAME_FMT = "<I4f64h64B4BhB"
FRAME_SIZE = struct.calcsize(FRAME_FMT)
assert FRAME_SIZE == 219, FRAME_SIZE

FLAG_NAMES = {
    0x01: "STATIONARY",
    0x02: "SENSOR_BLOCKED",
    0x04: "MOSTLY_INVALID",
    0x08: "YAW_SLEWING",
    0x10: "DROPOFF",
    0x20: "HEAD_OBSTACLE",
}

LOGS_DIR = Path(__file__).resolve().parent.parent / "Logs"


def decode_frame(raw: bytes) -> dict:
    v = struct.unpack(FRAME_FMT, raw)
    flags = v[-1]
    return {
        "ts": v[0],
        "quat": [round(x, 4) for x in v[1:5]],
        "distances": list(v[5:69]),
        "status": list(v[69:133]),
        "urg": list(v[133:137]),
        "near": v[137],
        "flags": flags,
        "flag_names": [n for bit, n in FLAG_NAMES.items() if flags & bit],
    }


class Reassembler:
    """Rebuilds frames from [hdr: seq<<4 | idx][payload] notifications."""

    def __init__(self) -> None:
        self.buf: bytearray | None = None
        self.seq = -1
        self.expect = 0
        self.dropped = 0

    def feed(self, data: bytes) -> bytes | None:
        if not data:
            return None
        seq, idx, payload = data[0] >> 4, data[0] & 0x0F, data[1:]
        if idx == 0:
            if self.buf is not None:
                self.dropped += 1          # previous frame never completed
            self.buf, self.seq, self.expect = bytearray(payload), seq, 1
        elif self.buf is not None and seq == self.seq and idx == self.expect:
            self.buf += payload
            self.expect += 1
        else:
            if self.buf is not None:
                self.dropped += 1
            self.buf = None
            return None
        if len(self.buf) >= FRAME_SIZE:
            frame, self.buf = bytes(self.buf[:FRAME_SIZE]), None
            return frame
        return None


class Receiver:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.reasm = Reassembler()
        self.count = 0
        self.rate_count = 0
        self.rate_t0 = time.monotonic()
        self.rate_hz = 0.0
        self.log = None
        if not args.no_log:
            LOGS_DIR.mkdir(exist_ok=True)
            path = LOGS_DIR / f"ble_{dt.datetime.now():%Y%m%d_%H%M%S}.jsonl"
            self.log = path.open("w", buffering=1)
            print(f"Logging frames to {path}")

    def on_notify(self, _char, data: bytearray) -> None:
        raw = self.reasm.feed(bytes(data))
        if raw is None:
            return
        f = decode_frame(raw)
        host_t = time.time()
        self.count += 1
        self.rate_count += 1
        now = time.monotonic()
        if now - self.rate_t0 >= 1.0:
            self.rate_hz = self.rate_count / (now - self.rate_t0)
            self.rate_count, self.rate_t0 = 0, now

        if self.log:
            self.log.write(json.dumps({"host_t": round(host_t, 4), **f}) + "\n")

        if self.count % self.args.print_every == 0:
            flags = ",".join(f["flag_names"]) or "-"
            print(
                f"#{self.count:6d} ts={f['ts']:9d} {self.rate_hz:5.1f} Hz "
                f"urg={f['urg']} near={f['near']:5d} flags=0x{f['flags']:02x}({flags}) "
                f"q={f['quat']} drop={self.reasm.dropped}"
            )

    async def find(self):
        name = self.args.name
        print(f"Scanning for {name} / {SVC_UUID} ...")
        return await BleakScanner.find_device_by_filter(
            lambda d, adv: SVC_UUID in [u.lower() for u in adv.service_uuids]
            or (d.name or adv.local_name) == name,
            timeout=self.args.scan_timeout,
        )

    async def run_once(self) -> None:
        dev = await self.find()
        if dev is None:
            print("Pin not found; retrying.")
            return
        print(f"Found {dev.name} [{dev.address}], connecting ...")
        lost = asyncio.Event()
        async with BleakClient(dev, disconnected_callback=lambda _c: lost.set()) as client:
            print(f"Connected (MTU {client.mtu_size}). Subscribing ...")
            await client.start_notify(FRAME_UUID, self.on_notify)
            print("Receiving frames. Ctrl+C to stop.")
            await lost.wait()
        print("Disconnected; reconnecting.")

    async def run(self) -> None:
        try:
            while True:
                try:
                    await self.run_once()
                except Exception as e:  # BLE errors are common; keep trying
                    print(f"BLE error: {e!r}; retrying in 2 s")
                await asyncio.sleep(2)
        finally:
            if self.log:
                self.log.close()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", default=DEFAULT_NAME, help="advertised device name")
    p.add_argument("--print-every", type=int, default=10, metavar="N",
                   help="print a summary every N frames (default 10)")
    p.add_argument("--scan-timeout", type=float, default=10.0)
    p.add_argument("--no-log", action="store_true", help="don't write a log file")
    args = p.parse_args()
    args.print_every = max(1, args.print_every)
    try:
        asyncio.run(Receiver(args).run())
    except KeyboardInterrupt:
        print(f"\nStopped.")


if __name__ == "__main__":
    main()

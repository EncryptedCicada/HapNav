#!/usr/bin/env python3
"""
Dual-port serial logger for HapNav.

Reads UART output from the wristband (nRF52840 USB-CDC, typically
/dev/ttyACM0) and the chest pin (ESP32-S3 USB-Serial-JTAG, typically
/dev/ttyACM1) concurrently, time-stamps each line on the *host* clock,
interleaves them chronologically, and writes the merged stream both to
stdout and to a fresh file under Logs/ at the project root.

Why host-side timestamping rather than the device timestamps:
the wristband's k_uptime_get_32() and the pin's k_uptime_get_32() count
from their respective boot moments, so they can't be aligned against
each other directly. The host's wall clock provides a single reference
that both UARTs land in as soon as the USB-CDC bytes arrive — close
enough to "the time the line was emitted" for any practical debugging
(USB-CDC adds ~ 1 ms of latency that is identical on both sides).

Usage:
    ./serial_logger.py                          # uses ACM0 (wrist), ACM1 (pin)
    ./serial_logger.py --wrist /dev/ttyACM2 --pin /dev/ttyACM3
    ./serial_logger.py --baud 921600

The script auto-reconnects when a port disappears (re-flash, replug).
Ctrl+C closes ports and the log file cleanly.
"""
from __future__ import annotations

import argparse
import datetime as dt
import os
import queue
import signal
import sys
import threading
import time
from pathlib import Path

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial not installed. Run: pip install pyserial")


PROJECT_ROOT = Path(__file__).resolve().parent.parent
LOGS_DIR = PROJECT_ROOT / "Logs"


def open_port_with_retry(path: str, baud: int, source: str,
                         out_q: "queue.Queue[tuple]",
                         stop: threading.Event) -> serial.Serial | None:
    """Try to open `path` until it succeeds or stop is signalled."""
    backoff = 0.5
    last_state = "open"   # to dedup "waiting…" lines
    while not stop.is_set():
        try:
            port = serial.Serial(path, baud, timeout=0.1)
            if last_state != "open":
                out_q.put((time.time(), "LOG ",
                           f"reconnected to {path} ({source})"))
            else:
                out_q.put((time.time(), "LOG ",
                           f"opened {path} ({source}) at {baud} baud"))
            return port
        except (serial.SerialException, FileNotFoundError, PermissionError) as e:
            if last_state == "open":
                out_q.put((time.time(), "LOG ",
                           f"waiting for {path} ({source}): {e}"))
                last_state = "waiting"
            stop.wait(backoff)
            backoff = min(backoff * 1.5, 5.0)
    return None


def reader_thread(path: str, baud: int, source: str,
                  out_q: "queue.Queue[tuple]",
                  stop: threading.Event) -> None:
    """Open the serial port, read lines, push (host_time, source, line)."""
    while not stop.is_set():
        port = open_port_with_retry(path, baud, source, out_q, stop)
        if port is None:
            return

        buf = bytearray()
        try:
            while not stop.is_set():
                chunk = port.read(256)
                if not chunk:
                    continue
                buf.extend(chunk)
                # Split out completed lines on either CR or LF (we get
                # both from Zephyr depending on the backend), tag each
                # with the host's wall clock.
                while True:
                    nl = -1
                    for sep in (b"\r\n", b"\n", b"\r"):
                        i = buf.find(sep)
                        if i >= 0 and (nl < 0 or i < nl):
                            nl = i
                            nl_len = len(sep)
                    if nl < 0:
                        break
                    line = buf[:nl].decode("utf-8", errors="replace")
                    del buf[: nl + nl_len]
                    if line:
                        out_q.put((time.time(), source, line))
        except (serial.SerialException, OSError) as e:
            out_q.put((time.time(), "LOG ",
                       f"{path} ({source}) dropped: {e}; reopening…"))
            try:
                port.close()
            except Exception:
                pass
            # Fall back to the outer reconnect loop.
            continue


def writer_thread(out_q: "queue.Queue[tuple]",
                  log_path: Path,
                  stop: threading.Event) -> None:
    """Pull from the shared queue and mirror to file + stdout."""
    with log_path.open("w", buffering=1) as f:   # line-buffered
        header = (f"# HapNav dual-port serial log\n"
                  f"# opened at {dt.datetime.now().isoformat(timespec='milliseconds')}\n"
                  f"# columns: host_time  source  line\n")
        f.write(header)
        sys.stdout.write(header)
        sys.stdout.flush()

        while not stop.is_set() or not out_q.empty():
            try:
                ts, source, line = out_q.get(timeout=0.2)
            except queue.Empty:
                continue
            stamp = dt.datetime.fromtimestamp(ts).strftime("%H:%M:%S.%f")[:-3]
            formatted = f"{stamp}  {source:5s}  {line}\n"
            f.write(formatted)
            sys.stdout.write(formatted)
            sys.stdout.flush()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Time-merged dual-port serial logger for HapNav.")
    parser.add_argument("--wrist", default="/dev/ttyACM0",
                        help="serial port for the wristband (default: ACM0)")
    parser.add_argument("--pin",   default="/dev/ttyACM1",
                        help="serial port for the chest pin (default: ACM1)")
    parser.add_argument("--baud",  type=int, default=115200,
                        help="baud rate for both ports (default: 115200; "
                             "ignored by USB-CDC but pyserial still needs a value)")
    parser.add_argument("--logs-dir", default=str(LOGS_DIR),
                        help=f"output directory (default: {LOGS_DIR})")
    args = parser.parse_args()

    logs_dir = Path(args.logs_dir)
    logs_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = logs_dir / f"serial_{stamp}.log"

    out_q: "queue.Queue[tuple]" = queue.Queue()
    stop = threading.Event()

    threads = [
        threading.Thread(target=reader_thread,
                         args=(args.wrist, args.baud, "WRIST", out_q, stop),
                         daemon=True, name="reader_wrist"),
        threading.Thread(target=reader_thread,
                         args=(args.pin,   args.baud, "PIN  ", out_q, stop),
                         daemon=True, name="reader_pin"),
        threading.Thread(target=writer_thread,
                         args=(out_q, log_path, stop),
                         daemon=False, name="writer"),
    ]

    def shutdown(*_args) -> None:
        if not stop.is_set():
            print("\n[shutting down — closing ports and flushing log…]",
                  file=sys.stderr)
        stop.set()

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f"Logging to {log_path}", file=sys.stderr)
    print(f"  WRIST = {args.wrist}", file=sys.stderr)
    print(f"  PIN   = {args.pin}",   file=sys.stderr)
    print(f"  baud  = {args.baud}",  file=sys.stderr)
    print(f"  Ctrl+C to stop.",      file=sys.stderr)

    for t in threads:
        t.start()

    # Block on the writer so the file is flushed before exit.
    threads[-1].join()
    return 0


if __name__ == "__main__":
    sys.exit(main())

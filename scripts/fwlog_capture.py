#!/usr/bin/env python3
"""
fwlog_capture.py - capture FWLOG filament-sensor data from the printer over USB

The printer firmware (6.3.3-SENS1+) streams one CSV row per planner block as
    FWLOG,time_ms,x_mm,y_mm,e_factor,msteps_raw,msteps_corr,axis_a_mm,axis_b_mm,ellipse_area_mm2,z_mm,block_seq,cum_e_mm,flags
while logging is active (M407 L1 ... M407 L0/D, injected by segment_gcode.py).
Firmware 10113+ appends z_mm/block_seq/cum_e_mm/flags; older 9-column rows are
still accepted (the CSV header adapts to whichever width the firmware sends).

Modes:
  --list                     list available serial ports (find the printer)
  --port COM5 --name NAME    live capture into NAME_raw.log (everything the
                             printer says) + NAME_fwlog.csv (FWLOG data only,
                             written progressively so a crash loses nothing)
  --filter-only FILE         recover: filter FWLOG rows out of an existing
                             raw log into a CSV (no serial port needed)
  --send "M407"              (repeatable) G-code lines sent once after the
                             port opens, e.g. pre-checks before the print

Ctrl-C stops the capture and prints a summary.

Requires: pyserial  (pip install pyserial)
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime

CSV_HEADER = ["time_ms", "x_mm", "y_mm", "e_factor", "msteps_raw",
              "msteps_corr", "axis_a_mm", "axis_b_mm", "ellipse_area_mm2",
              "z_mm", "block_seq", "cum_e_mm", "flags"]
MIN_FIELDS = 9  # firmware <= 10112 sends exactly the first 9 columns


def header_for(n_fields):
    """Column names for a data row of the given width (9 = old, 13 = 10113+)."""
    if n_fields < len(CSV_HEADER):
        return CSV_HEADER[:n_fields]
    return CSV_HEADER + [f"col{i}" for i in range(len(CSV_HEADER), n_fields)]


def bail(msg):
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def list_ports():
    try:
        from serial.tools.list_ports import comports
    except ImportError:
        bail("pyserial is not installed:  pip install pyserial")
    ports = comports()
    if not ports:
        print("No serial ports found.")
        return
    print(f"{'PORT':<10} {'DESCRIPTION':<40} HWID")
    for p in sorted(ports, key=lambda x: x.device):
        mark = "  <-- likely printer" if "prusa" in (p.description or "").lower() else ""
        print(f"{p.device:<10} {(p.description or '')[:40]:<40} {p.hwid}{mark}")


class FwlogFilter:
    """Parses FWLOG lines; writes data rows to CSV and collects counters."""

    def __init__(self, csv_path):
        self.entries = 0
        self.warnings = []           # raw warn lines
        self.dropped_total = 0
        self.stopped_info = None
        self.dump_info = None
        self._csv = open(csv_path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._csv)
        self._header_written = False  # written from the first data row's width

    def feed(self, line):
        """One stripped line from the printer. Returns True if it was FWLOG."""
        if not line.startswith("FWLOG,"):
            return False
        payload = line[len("FWLOG,"):]
        if payload.startswith("warn,"):
            self.warnings.append(payload)
            try:
                self.dropped_total += int(payload.rsplit("=", 1)[1])
            except (ValueError, IndexError):
                pass
        elif payload.startswith("logging_stopped,"):
            self.stopped_info = payload
        elif payload.startswith("dump_complete,"):
            self.dump_info = payload
        elif payload.startswith("time_ms,"):
            pass  # firmware CSV header echo
        else:
            row = payload.split(",")
            if len(row) >= MIN_FIELDS:
                if not self._header_written:
                    self._w.writerow(header_for(len(row)))
                    self._header_written = True
                self._w.writerow(row)
                self.entries += 1
            else:
                self.warnings.append(f"malformed({len(row)} fields): {payload[:60]}")
        return True

    def close(self):
        self._csv.close()


def filter_only(raw_path, csv_path):
    filt = FwlogFilter(csv_path)
    with open(raw_path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            filt.feed(line.rstrip("\r\n"))
    filt.close()
    print(f"[fwlog] {filt.entries} entries -> {csv_path}")
    if filt.dropped_total:
        print(f"[fwlog] WARNING: {filt.dropped_total} dropped entries "
              f"(log buffer overflow on the printer!)")
    return filt


def capture(args):
    try:
        import serial
    except ImportError:
        bail("pyserial is not installed:  pip install pyserial")

    os.makedirs(args.out, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = args.name or f"session_{stamp}"
    raw_path = os.path.join(args.out, f"{name}_raw.log")
    csv_path = os.path.join(args.out, f"{name}_fwlog.csv")

    filt = FwlogFilter(csv_path)
    raw = open(raw_path, "w", encoding="utf-8", newline="\n")
    t0 = time.monotonic()
    print(f"[fwlog] opening {args.port} @ {args.baud} ...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except serial.SerialException as e:
        bail(f"cannot open {args.port}: {e}\n"
             f"       Run with --list to see available ports.")

    try:
        for g in args.send or []:
            print(f"[fwlog] > {g}")
            ser.write((g + "\n").encode("ascii"))
        print(f"[fwlog] capturing -> {raw_path}")
        print(f"[fwlog] FWLOG csv  -> {csv_path}")
        print("[fwlog] Ctrl-C to stop.\n")
        next_report = 100
        stopped_reported = False
        while True:
            data = ser.readline()
            if data:
                text = data.decode("utf-8", errors="replace").rstrip("\r\n")
                raw.write(text + "\n")
                if filt.feed(text) and filt.entries >= next_report:
                    print(f"[fwlog]   {filt.entries} entries "
                          f"({time.monotonic() - t0:.0f}s)")
                    next_report += 100
                if filt.stopped_info and not stopped_reported:
                    print(f"[fwlog] printer stopped logging: {filt.stopped_info}")
                    print("[fwlog] (you can stop the capture with Ctrl-C now)")
                    stopped_reported = True
    except KeyboardInterrupt:
        pass
    finally:
        dur = time.monotonic() - t0
        ser.close()
        raw.close()
        filt.close()
        print(f"\n[fwlog] stopped after {dur:.0f}s")
        print(f"[fwlog] entries captured : {filt.entries}")
        print(f"[fwlog] raw log          : {raw_path}")
        print(f"[fwlog] csv              : {csv_path}")
        if filt.dropped_total:
            print(f"[fwlog] WARNING: {filt.dropped_total} dropped entries "
                  f"(printer log buffer overflow) - see raw log")
        if filt.warnings:
            print(f"[fwlog] {len(filt.warnings)} warn lines (first: {filt.warnings[0]})")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--port", help="serial port of the printer, e.g. COM5")
    ap.add_argument("--baud", type=int, default=115200,
                    help="baud rate (USB-CDC ignores it; default 115200)")
    ap.add_argument("--out", default="testprint_sessions",
                    help="output directory (default: testprint_sessions)")
    ap.add_argument("--name", help="session name used for file names "
                    "(default: session_<timestamp>)")
    ap.add_argument("--send", action="append",
                    help="G-code line to send once after opening "
                    "(repeatable), e.g. --send M407")
    ap.add_argument("--filter-only", metavar="RAWLOG",
                    help="filter FWLOG rows out of an existing raw log "
                    "into a CSV; no serial port needed")
    args = ap.parse_args()

    if args.list:
        list_ports()
        return
    if args.filter_only:
        base = os.path.splitext(args.filter_only)[0]
        filter_only(args.filter_only, base + "_fwlog.csv")
        return
    if not args.port:
        ap.error("--port is required for live capture (or use --list / --filter-only)")
    capture(args)


if __name__ == "__main__":
    main()

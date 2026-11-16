#!/usr/bin/env python3
"""
test_segment_gcode.py - host-side test suite for scripts/segment_gcode.py

Run from the repo root (or anywhere):
    python scripts/test_segment_gcode.py

Groups:
  H  - helper functions (param parsing, comment stripping, formatting,
       segment count computation)
  S  - segmentation behaviour (all four modal mode combinations, G92
       tracking, guards, position drift, stats)
  I  - sensor logging G-code injection (M407 L1/M406/M407 L0/D placement,
       print-line classification, EOF flush, full-print mode)
  C  - CLI subprocess test (in-place rewrite, stats header, exit codes)

Exit code 0 = all checks passed.
"""

import os
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import segment_gcode as sg  # noqa: E402

PASS = 0
FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ok    {name}")
    else:
        FAIL += 1
        print(f"  FAIL  {name}  {detail}")


def run(lines, **kw):
    """process_gcode with defaults matching segment_gcode CLI defaults."""
    kw.setdefault('max_seg_mm', 25.0)
    kw.setdefault('min_seg_mm', 5.0)
    kw.setdefault('min_e_per_seg', 1.0)
    kw.setdefault('sensor_lines', 0)
    kw.setdefault('correction_lines', 0)
    out, stats = sg.process_gcode(lines, **kw)
    return [l.rstrip('\n') for l in out], stats


def moves(out):
    """Return only G0/G1 lines of the output."""
    return [l for l in out if l[:2] in ('G0', 'G1', 'g0', 'g1')]


# ===========================================================================
print("== H: helpers ==")

# H1 basic parameter parsing
p = sg.parse_gcode_params("G1 X100.5 Y50 E1.234 F3600")
check("H1 parse basic", p == {'X': 100.5, 'Y': 50.0, 'E': 1.234, 'F': 3600.0}, str(p))

# H2 negative values
p = sg.parse_gcode_params("G1 X-10.5 E-0.8")
check("H2 parse negative", p == {'X': -10.5, 'E': -0.8}, str(p))

# H3 only present params
p = sg.parse_gcode_params("G1 F1800")
check("H3 parse partial", p == {'F': 1800.0}, str(p))

# H4 inline comments cannot inject fake parameters (regression: whole-line scan)
p = sg.parse_gcode_params("G1 X10 F3000 ; Y200 E1.5 reference point")
check("H4 comment stripped in parse", p == {'X': 10.0, 'F': 3000.0}, str(p))

# H5 strip_comment helper
check("H5a strip_comment", sg.strip_comment("G1 X1 ; note") == "G1 X1 ")
check("H5b strip_comment none", sg.strip_comment("G1 X1") == "G1 X1")
check("H5c strip_comment only", sg.strip_comment("; full comment") == "")

# H6 format_coord
check("H6a fmt trims zeros", sg.format_coord(100.000) == "100.0", sg.format_coord(100.0))
check("H6b fmt fractional", sg.format_coord(25.500) == "25.5", sg.format_coord(25.5))
check("H6c fmt keeps decimals", sg.format_coord(0.123456) == "0.12346", sg.format_coord(0.123456))

# H7 is_print_line classification
check("H7a print line XY+E", sg.is_print_line({'X': 1.0, 'E': 0.5}, 0.5))
check("H7b not: X only", not sg.is_print_line({'X': 1.0}, 0.0))
check("H7c not: E only", not sg.is_print_line({'E': 0.5}, 0.5))
check("H7d not: retraction", not sg.is_print_line({'X': 1.0, 'E': -0.5}, -0.5))

# H8 compute_segment_count decision table
cases = [
    # (xy, de, max, min, min_e, expected)
    (10.0, 0.4, 25.0, 5.0, 1.0, 1),    # short -> untouched
    (100.0, 8.0, 25.0, 5.0, 1.0, 4),   # 100/25 -> 4, E 2.0/seg ok
    (50.0, 4.0, 25.0, 5.0, 1.0, 2),    # exact multiple
    (100.0, 2.0, 25.0, 5.0, 1.0, 2),   # E guard: 4 segs would be 0.5 -> reduce to 2
    (60.0, 0.5, 25.0, 5.0, 1.0, 1),    # E guard collapses to 1 (0.5 < 1.0 even whole)
    (100.0, 0.0, 25.0, 5.0, 1.0, 4),   # travel move: E guard must NOT apply
    (24.9, 9.0, 25.0, 5.0, 1.0, 1),    # just under limit
]
for i, (xy, de, mx, mn, me, exp) in enumerate(cases):
    n = sg.compute_segment_count(xy, de, mx, mn, me)
    check(f"H8-{i} count xy={xy} de={de}", n == exp, f"got {n}, want {exp}")

# ===========================================================================
print("== S: segmentation ==")

# S1 absolute XYZ + absolute E, 100mm X move with E8 (E guard allows 4 x 2.0)
out, st = run(["G90\n", "M82\n", "G1 X100 E8 F3000\n"])
m = moves(out)
check("S1a split into 4", len(m) == 4, str(m))
check("S1b endpoints", [sg.parse_gcode_params(l).get('X') for l in m] == [25.0, 50.0, 75.0, 100.0], str(m))
check("S1c E cumulative", [sg.parse_gcode_params(l).get('E') for l in m] == [2.0, 4.0, 6.0, 8.0], str(m))
check("S1d F only on first", 'F' in m[0] and all('F' not in l for l in m[1:]), str(m))
check("S1e stats", st['segmented_moves'] == 1 and st['segments_created'] == 4, str(st))

# S1b-E same move with only E2: guard reduces 4 -> 2 segments (1.0 mm E each)
out, st = run(["G90\n", "M82\n", "G1 X100 E2 F3000\n"])
m = moves(out)
check("S1f E-guard reduces to 2", len(m) == 2
      and [sg.parse_gcode_params(l).get('E') for l in m] == [1.0, 2.0], str(m))

# S2 relative XYZ + relative E
out, st = run(["G91\n", "M83\n", "G1 X100 E8\n"])
m = moves(out)
check("S2a 4 segments", len(m) == 4, str(m))
check("S2b rel X 25 each", all(sg.parse_gcode_params(l).get('X') == 25.0 for l in m), str(m))
check("S2c rel E 2 each", all(sg.parse_gcode_params(l).get('E') == 2.0 for l in m), str(m))

# S3 PrusaSlicer MK4 default: absolute XYZ + relative E
out, st = run(["G90\n", "M83\n", "G1 X0 Y0\n", "G1 X100 E8\n"])
m = moves(out)
check("S3a 4 segments", len(m) == 5, str(m))
check("S3b X absolute", [sg.parse_gcode_params(l).get('X') for l in m[1:]] == [25.0, 50.0, 75.0, 100.0], str(m))
check("S3c E relative 2 each", all(sg.parse_gcode_params(l).get('E') == 2.0 for l in m[1:]), str(m))

# S4 G92 E0 mid-file keeps later absolute-E deltas correct
out, st = run(["G90\n", "M82\n",
               "G1 X50 E1\n",       # 50mm, de=1: E guard -> 1 segment (0.5/seg < 1.0)
               "G92 E0\n",          # reset E origin
               "G1 X150 E1\n"])     # de must again be 1 (not 2, not 0)
m = moves(out)
check("S4a G92 keeps delta", m == ["G1 X50 E1", "G1 X150 E1"], str(m))

# S5 pure E (retraction/prime) and pure Z (layer change) never split
src = ["G90\n", "M83\n", "G1 E-0.8 F2400\n", "G1 Z0.4 F1000\n"]
out, st = run(src)
check("S5 pass-through unchanged", out == [l.strip() for l in src], str(out))

# S6 travel move (G0, no E) still split
out, st = run(["G90\n", "G0 X100 F7200\n"])
m = moves(out)
check("S6 travel split", len(m) == 4 and m[0].startswith('G0'), str(m))

# S7 short move untouched, byte-for-byte
src = ["G90\n", "M83\n", "G1 X10 Y5 E0.4 ; TYPE:Perimeter\n"]
out, st = run(src)
check("S7 short untouched", out == [l.strip() for l in src], str(out))

# S8 XY+Z move: Z interpolated across segments
out, st = run(["G90\n", "G1 X100 Z0.4 E8\n"])
zs = [sg.parse_gcode_params(l).get('Z') for l in moves(out)]
check("S8 Z interpolated", zs == [0.1, 0.2, 0.3, 0.4], str(zs))

# S9 no cumulative position drift after many segmented moves
out, st = run(["G90\n", "M82\n"] +
              [f"G1 X{x} E{x / 25}\n" for x in range(100, 1100, 100)])
last = moves(out)[-1]
check("S9 final X exact", sg.parse_gcode_params(last).get('X') == 1000.0, last)

# S10 lowercase g1 handled
out, st = run(["g1 x100 e8\n"])
check("S10 lowercase split", len(moves(out)) == 4, str(out))

# S11 line with only F passes through
src = ["G90\n", "G1 F1800\n"]
out, st = run(src)
check("S11 F-only passthrough", out == ["G90", "G1 F1800"], str(out))

# ===========================================================================
print("== I: sensor logging injection ==")

# I1 default 16/8 style: L1 before line 1, M406 after line 8, L0+D after 16
gcode = ["G90\n", "M83\n"] + [f"G1 X{i * 10} Y0 E0.4\n" for i in range(1, 21)]
out, st = run(gcode, sensor_lines=16, correction_lines=8)
flat = '\n'.join(out)
check("I1a M407 L1 present once", flat.count("M407 L1") == 1)
check("I1b M406 present once (mid)", flat.count("M406") == 1)
check("I1c M407 L0 present", flat.count("M407 L0") == 1)
check("I1d M407 D present", flat.count("M407 D") == 1)
idx = {cmd: out.index(cmd) for cmd in ("M407 L1", "M406", "M407 L0")}
check("I1e order L1 < M406 < L0", idx["M407 L1"] < idx["M406"] < idx["M407 L0"], str(idx))
# count print lines around injections: M406 must come after the 8th, L0 after the 16th
def count_print_lines_before(pos):
    n = 0
    for l in out[:pos]:
        if l.startswith('G1'):
            prm = sg.parse_gcode_params(l)
            if sg.is_print_line(prm, prm.get('E', 0.0)):
                n += 1
    return n
check("I1f M406 after 8 print lines", count_print_lines_before(idx["M406"]) == 8,
      str(count_print_lines_before(idx["M406"])))
check("I1g L0 after 16 print lines", count_print_lines_before(idx["M407 L0"]) == 16,
      str(count_print_lines_before(idx["M407 L0"])))

# I2 retractions and travels do not count as print lines
gcode = ["G90\n", "M83\n",
         "G1 E-0.8 F2400\n",   # retraction: not counted
         "G0 X50 Y50\n",       # travel: not counted
         "G1 E0.8 F2400\n",    # prime: not counted (no XY)
         "G1 X60 E0.4\n"]      # first real print line
out, st = run(gcode, sensor_lines=5, correction_lines=0)
check("I2 L1 injected right before first print line",
      out[out.index("M407 L1") + 1] == "G1 X60 E0.4", str(out))

# I3 sensor_lines=0 -> no injection at all
out, st = run(["G90\n", "M83\n", "G1 X10 E0.4\n"], sensor_lines=0)
check("I3 no injections", not any('M407' in l or 'M406' in l for l in out), str(out))

# I4 correction_lines=0 -> M406 only at the stop point
out, st = run(["G90\n", "M83\n"] + [f"G1 X{i} E0.4\n" for i in range(1, 5)],
              sensor_lines=3, correction_lines=0)
flat = '\n'.join(out)
check("I4 M406 after stop", out.index("M407 L0") < flat.splitlines().index("M406"), flat)

# I5 correction_lines >= sensor_lines -> treated as "keep correction on", M406 at stop
out, st = run(["G90\n", "M83\n"] + [f"G1 X{i} E0.4\n" for i in range(1, 5)],
              sensor_lines=3, correction_lines=99)
flat = '\n'.join(out)
check("I5 same as correction 0", flat.count("M406") == 1 and "M407 L0" in flat, flat)

# I6 EOF flush when fewer print lines than sensor_lines exist
out, st = run(["G90\n", "M83\n", "G1 X10 E0.4\n", "G1 X20 E0.4\n"],
              sensor_lines=10, correction_lines=0)
check("I6 EOF flush", out[-1] == "M407 D" and out[-2] == "M407 L0"
      and "EOF" in out[-3], str(out[-3:]))

# I7 no print lines at all -> no logging start, no EOF flush
out, st = run(["G90\n", "G0 X100\n", "G1 E-1\n"], sensor_lines=10)
check("I7 nothing injected", not any('M407' in l for l in out), str(out))

# I8 full-print mode (huge sensor_lines, correction 0): L1 only + EOF stop, no M406
out, st = run(["G90\n", "M83\n"] + [f"G1 X{i * 10} E0.4\n" for i in range(1, 10)],
              sensor_lines=1000000, correction_lines=0)
flat = '\n'.join(out)
check("I8a L1 once", flat.count("M407 L1") == 1)
check("I8b no M406", "M406" not in flat)
check("I8c EOF stop", "M407 L0" in flat and "M407 D" in flat)

# I9 logging-only mode (huge sensor_lines, correction_lines=1): M406 after line 1
out, st = run(["G90\n", "M83\n"] + [f"G1 X{i * 10} E0.4\n" for i in range(1, 10)],
              sensor_lines=1000000, correction_lines=1)
flat = '\n'.join(out)
i_g1 = out.index("G1 X10 E0.4")
check("I9 M406 after first line", flat.count("M406") == 1
      and out[i_g1 + 1].startswith(';') and out[i_g1 + 2] == "M406", flat)

# I10 print lines counted per ORIGINAL line, not per segment
out, st = run(["G90\n", "M83\n", "G1 X100 E8\n"],  # 100mm -> 4 segments, still line #1
              sensor_lines=1, correction_lines=0)
check("I10 stop after all segments of line 1",
      out[-1] == "M406" and out[-2] == "M407 D" and moves(out)[-1] == "G1 X100.0 E2.0",
      str(out[-4:]))

# I11 injection also works in fully relative mode
out, st = run(["G91\n", "M83\n", "G1 X100 Y0 E2\n"],
              sensor_lines=1, correction_lines=0)
check("I11 relative injection", "M407 L1" in out and "M407 L0" in out, str(out))

# I12 after stop, further print lines are not counted/injected again
gcode = (["G90\n", "M83\n"] + [f"G1 X{i} E0.4\n" for i in range(1, 4)]
         + ["G0 X0 Y0\n"] + [f"G1 X{i} E0.4\n" for i in range(10, 13)])
out, st = run(gcode, sensor_lines=3, correction_lines=0)
flat = '\n'.join(out)
check("I12 single stop sequence", flat.count("M407 L0") == 1 and flat.count("M407 D") == 1)

# ===========================================================================
print("== C: CLI subprocess ==")

sample = ("; test file\n"
          "G90\nM83\n"
          "G1 X100 E8 F3000\n"
          "G1 X0 E8\n")

with tempfile.TemporaryDirectory() as td:
    g = os.path.join(td, "test.gcode")
    with open(g, 'w') as f:
        f.write(sample)
    r = subprocess.run([sys.executable, os.path.abspath(sg.__file__),
                        "--max-segment-mm", "25",
                        "--sensor-lines", "1000000", g],
                       capture_output=True, text=True)
    check("C1 exit code 0", r.returncode == 0, r.stderr)
    body = open(g).read()
    check("C2 stats header", "FILWIDTH SEGMENTER" in body, body[:200])
    check("C3 in-place rewritten", "M407 L1" in body and body.count("G1") >= 8, body)

    # invalid arguments must fail with exit code 1
    r = subprocess.run([sys.executable, os.path.abspath(sg.__file__),
                        "--max-segment-mm", "0.5", g],
                       capture_output=True, text=True)
    check("C4 invalid args rejected", r.returncode == 1, r.stdout + r.stderr)

    r = subprocess.run([sys.executable, os.path.abspath(sg.__file__),
                        "Z:/does/not/exist.gcode"],
                       capture_output=True, text=True)
    check("C5 missing file rejected", r.returncode == 1, r.stdout + r.stderr)

# ===========================================================================
print(f"\n{PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)

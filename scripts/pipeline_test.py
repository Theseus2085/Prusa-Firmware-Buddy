#!/usr/bin/env python3
"""
pipeline_test.py - End-to-end test of the filament width sensor pipeline.

Mirrors the EXACT algorithms from both repos (float32 arithmetic via numpy):

  Sensor module  (sensor-module/src/main.cpp, FW 0.7.0):
      mm -> x10000 -> 5 binary digit bytes -> + version byte + CRC8

  Printer firmware (lib/Marlin/Marlin/src/feature/filwidth.cpp):
      crc8_frame check, decode_axis_digits, margin gate, spatial queues,
      retraction debt, pop at nozzle, lookahead integral, ratio clamp.

Two models are simulated side by side on identical filament profiles:
  OLD : the pipeline BEFORE the 2026-10 changes (10-byte payload, no CRC,
        no margin gate, no ratio clamp) - the original I2C implementation.
  NEW : the CURRENT pipeline (12-byte frame w/ version+CRC8, margin gate,
        +/-20 % clamp, comm watchdog).

For valid input data the two must produce identical correction ratios.
For invalid data (out-of-margin spikes, corrupted frames) the NEW model
must demonstrably protect the print; the differences there are intended.
"""

import numpy as np
import sys

f32 = np.float32

# ---------------------------------------------------------------------------
# Constants from Configuration_MK4_adv.h / filwidth.h / filwidth.cpp
# ---------------------------------------------------------------------------
NOMINAL_MM      = f32(1.75)
MARGIN_MM       = f32(0.6)          # FILWIDTH_ERROR_MARGIN
MAX_RATIO_PCT   = f32(20.0)         # FILWIDTH_SENSOR_MAX_RATIO_PERCENT
SPACING_MM      = f32(73.8)         # FILWIDTH_SENSOR_SPACING_MM
OFFSET1_MM      = f32(140.0)        # MEASUREMENT_DELAY_CM(14) * 10
OFFSET2_MM      = f32(OFFSET1_MM + SPACING_MM)
QUEUE_BIN_MM    = f32(1.0)          # FILWIDTH_SENSOR_QUEUE_BIN_MM
MIN_VALID_X10000 = 5000             # 0.5 mm
MAX_VALID_X10000 = 35000            # 3.5 mm
PROTOCOL_VERSION = 0x01
MAX_CONSEC_FAILURES = 8
DIGITS = 5
AXES = 2
FPI = f32(np.pi)                    # (float)PI in Marlin

NOMINAL_AREA = f32(f32(f32(0.25) * FPI) * f32(NOMINAL_MM * NOMINAL_MM))  # get_area_mm2(nom, nom)


# ---------------------------------------------------------------------------
# Shared helpers (identical code on both sides of the wire)
# ---------------------------------------------------------------------------
def crc8(data: bytes) -> int:
    """CRC-8 poly 0x07 init 0x00 - crc8() [sensor] == crc8_frame() [printer]."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def get_area_mm2(a, b):
    """filwidth.cpp get_area_mm2: 0.25f * PI * major * minor"""
    return f32(f32(f32(f32(0.25) * FPI) * a) * b)


def clamp_ratio(ratio):
    """NEW only: clamp_ratio_percent"""
    return f32(min(max(ratio, -MAX_RATIO_PCT), MAX_RATIO_PCT))


# ---------------------------------------------------------------------------
# Sensor module model (main.cpp)
# ---------------------------------------------------------------------------
def sensor_mm_to_fixed(val):
    v = min(max(val, f32(0.0)), f32(9.9999))
    return int(f32(f32(v) * f32(10000.0)) + f32(0.5))


def sensor_format_digits(val_x10000, buf, off=0):
    val_x10000 = min(val_x10000, 99999)
    buf[off + 0] = (val_x10000 // 10000) % 10
    buf[off + 1] = (val_x10000 // 1000) % 10
    buf[off + 2] = (val_x10000 // 100) % 10
    buf[off + 3] = (val_x10000 // 10) % 10
    buf[off + 4] = val_x10000 % 10


def build_frame(a_mm, b_mm, version=PROTOCOL_VERSION, with_crc=True):
    """12-byte frame (FW 0.7.0). with_crc=False -> 10-byte legacy frame."""
    frame = bytearray(12)
    sensor_format_digits(sensor_mm_to_fixed(a_mm), frame, 0)
    sensor_format_digits(sensor_mm_to_fixed(b_mm), frame, 5)
    if with_crc:
        frame[10] = version
        frame[11] = crc8(frame[:11])
    return bytes(frame[:10] if not with_crc else frame)


# ---------------------------------------------------------------------------
# Printer firmware model (filwidth.cpp)
# ---------------------------------------------------------------------------
class PrinterModel:
    def __init__(self, margin_gate: bool, ratio_clamp: bool, check_crc: bool):
        self.margin_gate = margin_gate    # NEW behaviour
        self.ratio_clamp = ratio_clamp
        self.check_crc = check_crc
        cap = 302                          # FILWIDTH_SENSOR_QUEUE_CAPACITY (294+8)
        self.cap = cap
        self.queue   = [[f32(0)] * cap for _ in range(2)]
        self.targets = [[f32(0)] * cap for _ in range(2)]
        self.head = [0, 0]
        self.size = [0, 0]
        self.pos = f32(0.0)                # filament_position_mm
        self.debt = f32(0.0)               # retraction_debt_mm
        self.next_enqueue = f32(0.0)
        self.latest = [NOMINAL_MM, NOMINAL_MM]
        self.measured = NOMINAL_MM
        self.nominal_area = NOMINAL_AREA
        self.consec_failures = 0
        self.correction_enabled = True
        self.watchdog_tripped = False
        self.held_correction = False

    # -- decode path (update_from_sensor) -----------------------------------
    def decode_axis_digits(self, frame, off):
        value = 0
        collected = 0
        i = off
        while collected < DIGITS:
            v = frame[i]
            i += 1
            if v in (0x0A, 0x0D, 0x20) or v == ord('.'):
                continue
            if v <= 9:
                d = v
            elif 0x30 <= v <= 0x39:
                d = v - 0x30
            else:
                return None
            value = value * 10 + d
            collected += 1
        if not (MIN_VALID_X10000 <= value <= MAX_VALID_X10000):
            return None
        return value

    def handle_frame(self, frame: bytes):
        """Returns ('ok', a_mm, b_mm) or ('fail', reason)."""
        if self.check_crc:
            if len(frame) < 12 or frame[10] != PROTOCOL_VERSION:
                self._fail("bad_version"); return ('fail', 'bad_version')
            if frame[11] != crc8(frame[:11]):
                self._fail("crc"); return ('fail', 'crc_failed')
        a_raw = self.decode_axis_digits(frame, 0)
        b_raw = self.decode_axis_digits(frame, 5)
        if a_raw is None or b_raw is None:
            self._fail("decode"); return ('fail', 'decode_failed')

        # successful frame -> watchdog recovery
        if self.consec_failures:
            self.consec_failures = 0
        if self.watchdog_tripped:
            self.watchdog_tripped = False
            if self.held_correction:
                self.correction_enabled = True

        a_mm = f32(a_raw * 0.0001)
        b_mm = f32(b_raw * 0.0001)

        a_ok = b_ok = True
        if self.margin_gate:
            a_ok = (NOMINAL_MM - MARGIN_MM) <= a_mm <= (NOMINAL_MM + MARGIN_MM)
            b_ok = (NOMINAL_MM - MARGIN_MM) <= b_mm <= (NOMINAL_MM + MARGIN_MM)

        if f32(self.pos + f32(0.0001)) >= self.next_enqueue:
            if a_ok:
                self.enqueue(0, a_mm)
            if b_ok:
                self.enqueue(1, b_mm)
            self.process_ready()
            self.next_enqueue = f32(self.pos + QUEUE_BIN_MM)
        return ('ok', a_mm, b_mm)

    def _fail(self, reason):
        self.consec_failures += 1
        if self.consec_failures >= MAX_CONSEC_FAILURES and not self.watchdog_tripped:
            self.watchdog_tripped = True
            self.held_correction = self.correction_enabled
            self.correction_enabled = False
            for ax in range(2):            # clear_alignment_queues
                self.head[ax] = 0
                self.size[ax] = 0
            self.next_enqueue = self.pos

    # -- spatial queue (enqueue/try_pop/process_ready) -----------------------
    def enqueue(self, ax, d):
        tail = (self.head[ax] + self.size[ax]) % self.cap
        self.queue[ax][tail] = d
        self.targets[ax][tail] = f32(self.pos + (OFFSET1_MM if ax == 0 else OFFSET2_MM))
        if self.size[ax] < self.cap:
            self.size[ax] += 1
        else:
            self.head[ax] = (self.head[ax] + 1) % self.cap

    def try_pop(self, ax):
        if self.size[ax] == 0:
            return None
        h = self.head[ax]
        if self.pos < self.targets[ax][h]:
            return None
        d = self.queue[ax][h]
        self.head[ax] = (h + 1) % self.cap
        self.size[ax] -= 1
        return d

    def process_ready(self):
        updated = False
        for ax in (0, 1):
            while True:
                d = self.try_pop(ax)
                if d is None:
                    break
                self.latest[ax] = d
                updated = True
        if updated:
            prod = max(f32(self.latest[0] * self.latest[1]), f32(0.0001))
            self.measured = f32(np.sqrt(prod, dtype=f32))

    # -- motion (advance_e, I2C branch only - identical in OLD and NEW) ------
    def advance_e(self, e):
        if e < 0:
            self.debt = f32(self.debt + (-e))
        elif e > 0:
            paid = min(self.debt, e)
            self.debt = f32(self.debt - paid)
            fwd = f32(e - paid)
            if fwd > 0:
                self.pos = f32(self.pos + fwd)
                self.process_ready()

    # -- lookahead integral (get_averaged_size_ratio) -------------------------
    def sample_to_size_ratio(self):
        a = max(self.latest[0], f32(0.01))
        b = max(self.latest[1], f32(0.01))
        area = get_area_mm2(a, b)
        if self.nominal_area <= 0:
            return f32(0)
        r = f32(f32(f32(self.nominal_area / area) * f32(100.0)) - f32(100.0))
        return clamp_ratio(r) if self.ratio_clamp else r

    def get_averaged_size_ratio(self, extrude_mm):
        if self.nominal_area <= 0:
            return f32(0)
        if extrude_mm <= 0:
            return self.sample_to_size_ratio()

        sim_pos = self.pos
        end = f32(self.pos + extrude_mm)
        ha, hb = self.head[0], self.head[1]
        sa, sb = self.size[0], self.size[1]
        la, lb = self.latest[0], self.latest[1]
        total = f32(0.0)

        while sim_pos < end:
            ta = self.targets[0][ha] if sa > 0 else f32(end + f32(1.0))
            tb = self.targets[1][hb] if sb > 0 else f32(end + f32(1.0))
            nxt = min(end, ta, tb)
            area = get_area_mm2(max(la, f32(0.01)), max(lb, f32(0.01)))
            step = f32(nxt - sim_pos)
            if step <= 0:
                step = f32(0.0001)
            total = f32(total + f32(area * step))
            sim_pos = f32(sim_pos + step)
            if sa > 0 and sim_pos >= ta:
                la = self.queue[0][ha]; ha = (ha + 1) % self.cap; sa -= 1
            if sb > 0 and sim_pos >= tb:
                lb = self.queue[1][hb]; hb = (hb + 1) % self.cap; sb -= 1

        avg = f32(total / extrude_mm)
        if avg <= 0:
            return f32(0)
        r = f32(f32(f32(self.nominal_area / avg) * f32(100.0)) - f32(100.0))
        return clamp_ratio(r) if self.ratio_clamp else r


def make_old(): return PrinterModel(margin_gate=False, ratio_clamp=False, check_crc=False)
def make_new(): return PrinterModel(margin_gate=True,  ratio_clamp=True,  check_crc=True)


# ---------------------------------------------------------------------------
# Simulation driver
# ---------------------------------------------------------------------------
def run_scenario(profile, steps=420, retract_at=None, retract_mm=2.0, corrupt_at=None):
    """Feed 1 mm of filament per step; returns (old_ratios, new_ratios, models, new_states)."""
    old, new = make_old(), make_new()
    ratios_old, ratios_new, new_states = [], [], []
    for step in range(steps):
        # planner advances 1 mm of filament (one block of E=1mm)
        old.advance_e(f32(1.0))
        new.advance_e(f32(1.0))

        if corrupt_at is not None and step in corrupt_at:
            frame = bytearray(build_frame(profile(new.pos + OFFSET1_MM),
                                          profile(new.pos + OFFSET2_MM)))
            frame[1] = (frame[1] + 1) % 10          # flip a digit, DON'T fix CRC
            frame_old = bytes(frame[:10])
            new.handle_frame(bytes(frame))
            old.handle_frame(frame_old)
        else:
            a = profile(f32(new.pos + OFFSET1_MM))
            b = profile(f32(new.pos + OFFSET2_MM))
            new.handle_frame(build_frame(a, b, with_crc=True))
            old.handle_frame(build_frame(a, b, with_crc=False))

        if retract_at is not None and step == retract_at:
            for m in (old, new):                    # G10/G11 pair
                m.advance_e(f32(-retract_mm))
                m.advance_e(f32(+retract_mm))

        ratios_new.append(new.get_averaged_size_ratio(f32(1.0)))
        ratios_old.append(old.get_averaged_size_ratio(f32(1.0)))
        new_states.append((new.watchdog_tripped, new.correction_enabled, new.consec_failures))
    return ratios_old, ratios_new, old, new, new_states


def max_abs_diff(x, y):
    return max((abs(float(a) - float(b)) for a, b in zip(x, y)), default=0.0)


PASS, FAIL = "PASS", "FAIL"
results = []

def check(name, cond, detail=""):
    results.append((name, PASS if cond else FAIL, detail))
    print(f"  [{PASS if cond else FAIL}] {name}{(' - ' + detail) if detail else ''}")


# ---------------------------------------------------------------------------
print("=" * 74)
print("Pipeline equivalence test: OLD (pre-change) vs NEW (current) implementation")
print("=" * 74)

# --- T0: payload round trip (sensor encode -> printer decode) --------------
print("\nT0: Sensor payload round-trip (mm -> digits -> CRC -> decode)")
ok = True
for mm in (1.47, 1.5, 1.68, 1.75, 1.80, 1.99, 2.0, 2.35):
    frame = build_frame(mm, mm)
    dec = PrinterModel(True, True, True).decode_axis_digits
    a = dec(frame, 0)
    expect = round(mm * 10000)
    if a != expect:
        ok = False
        print(f"      mm={mm}: decoded x10000={a}, expected {expect}")
check("T0 round-trip exact for all test diameters", ok)
check("T0 CRC over identical bytes matches printer-side crc8_frame",
      crc8(build_frame(1.75, 1.75)[:11]) == build_frame(1.75, 1.75)[11])

# --- T1: constant nominal ----------------------------------------------------
print("\nT1: constant 1.75/1.75 mm - ratio must be 0 everywhere")
ro, rn, _, _, _ = run_scenario(lambda x: f32(1.75))
check("T1 OLD == NEW (identical sequences)", max_abs_diff(ro, rn) == 0.0)
check("T1 ratio == 0 in NEW", max(abs(float(r)) for r in rn) == 0.0,
      f"max |ratio| = {max(abs(float(r)) for r in rn):.2e} %")

# --- T2: diameter step 1.75 -> 1.80 (both axes) ------------------------------
print("\nT2: step to 1.80/1.80 at x=100 - analytic steady state -5.4784 %")
prof2 = lambda x: f32(1.75 if x < 100 else 1.80)
ro, rn, _, _, _ = run_scenario(prof2)
analytic = f32(f32(f32(1.75 * 1.75) / f32(1.80 * 1.80)) * f32(100.0)) - f32(100.0)
check("T2 OLD == NEW (identical sequences)", max_abs_diff(ro, rn) == 0.0,
      f"max diff = {max_abs_diff(ro, rn):.2e}")
final_new = float(rn[-1]); final_old = float(ro[-1])
check("T2 NEW steady state == analytic value", abs(final_new - float(analytic)) < 1e-3,
      f"new={final_new:.4f} %, analytic={float(analytic):.4f} %")
check("T2 OLD steady state == analytic value", abs(final_old - float(analytic)) < 1e-3,
      f"old={final_old:.4f} %")

# --- T3: ovality 1.70/1.80 ---------------------------------------------------
print("\nT3: ovality 1.70/1.80 (same area ~nominal) - small positive ratio")
prof3 = lambda x: f32(1.70 if x < 100 else 1.80)  # axis1/axis2 shifted by offset anyway
ro, rn, _, _, _ = run_scenario(lambda x: f32(1.70 if (x < 100) else 1.80))
check("T3 OLD == NEW (identical sequences)", max_abs_diff(ro, rn) == 0.0,
      f"max diff = {max_abs_diff(ro, rn):.2e}")

# --- T4: retraction / prime must not consume samples -------------------------
print("\nT4: retract 2 mm + prime 2 mm mid-print - ratio sequence must be identical")
ro_plain, rn_plain, _, _, _ = run_scenario(prof2)
ro_ret, rn_ret, _, _, _ = run_scenario(prof2, retract_at=250, retract_mm=2.0)
check("T4 NEW with retraction == NEW without retraction",
      max_abs_diff(rn_plain, rn_ret) == 0.0,
      f"max diff = {max_abs_diff(rn_plain, rn_ret):.2e}")
check("T4 OLD with retraction == OLD without retraction",
      max_abs_diff(ro_plain, ro_ret) == 0.0)

# --- T5: out-of-margin spike (intended behavioural difference) ---------------
print("\nT5: spike to 1.10/1.10 for 5 mm of filament (outside +/-0.6 mm margin)")
prof5 = lambda x: f32(1.10 if 150 <= x < 155 else 1.75)
ro, rn, _, _, _ = run_scenario(prof5)
old_peak = max(abs(float(r)) for r in ro)
new_peak = max(abs(float(r)) for r in rn)
# The 73.8 mm axis spacing means a 5 mm spike never hits both axes at once:
# the worst OLD block is 1.10 x 1.75 -> (1.75/1.10 - 1)*100 = +59.09 %
check("T5 OLD enforces the spike (peak ~+59 %, one axis)", 55.0 < old_peak < 63.0,
      f"old peak = {old_peak:+.1f} %")
check("T5 NEW rejects the spike and holds last good (peak ~0)", new_peak < 0.01,
      f"new peak = {new_peak:+.4f} %")

# --- T6: CRC watchdog (intended behavioural difference) ----------------------
print("\nT6: 10 corrupted frames (CRC) then recovery")
ro, rn, old_m, new_m, states = run_scenario(prof2, corrupt_at=set(range(200, 210)))
# OLD has no CRC: an in-range digit flip (e.g. 1.75 -> 1.85) is enqueued as truth.
old_spiked = any(abs(float(r) - 0.0) > 0.05 for r in ro[200:215])
check("T6 OLD applies corrupted in-range value as measurement", old_spiked,
      "old has no CRC -> flipped digit becomes a 'measurement'")
# NEW: frames 200..207 are the first 8 failures -> watchdog must trip exactly there.
tripped_at = next((i for i, (t, c, f) in enumerate(states) if t), None)
check("T6 NEW watchdog trips at 8th failure (correction off)",
      tripped_at == 207 and states[207][1] is False,
      f"tripped at step {tripped_at}, correction={states[207][1]}")
check("T6 NEW recovered on first valid frame",
      (not new_m.watchdog_tripped) and new_m.correction_enabled,
      f"final: watchdog_tripped={new_m.watchdog_tripped}, correction_enabled={new_m.correction_enabled}")

# --- T7: margin gate boundaries -------------------------------------------------
print("\nT7: Margin-Gate exakt an der Grenze (nominal +/- 0.6 mm)")
def t7_probe(mm):
    m7 = make_new()
    m7.advance_e(f32(1.0))
    m7.next_enqueue = f32(0.0)
    r7 = m7.handle_frame(build_frame(mm, mm, with_crc=True))
    return r7, m7.size[0]
r7, sz = t7_probe(f32(2.35))
check("T7 +0.6000 akzeptiert (WITHIN inklusiv)", r7[0] == 'ok' and sz == 1)
r7, sz = t7_probe(f32(1.15))
check("T7 -0.6000 akzeptiert", r7[0] == 'ok' and sz == 1)
r7, sz = t7_probe(f32(2.3601))
check("T7 +0.6001 verworfen (Hold-last-good)", r7[0] == 'ok' and sz == 0)
r7, sz = t7_probe(f32(0.5))
check("T7 0.50 verworfen", r7[0] == 'ok' and sz == 0)

# --- T8: ratio clamp boundaries --------------------------------------------------
print("\nT8: Ratio-Clamp exakt bei +/-20 %")
m = make_new()
m.latest = [f32(1.75 / 1.095445115), f32(1.75 / 1.095445115)]  # a=b -> +20 %
r = m.get_averaged_size_ratio(f32(0.0))
check("T8 +20 % wird exakt durchgereicht", abs(float(r) - 20.0) < 0.01, f"ratio={float(r):.4f}")
m.latest = [f32(1.5), f32(1.5)]           # +36.1 % -> kappen
r = m.get_averaged_size_ratio(f32(0.0))
check("T8 darueber auf +20 % gekappt", abs(float(r) - 20.0) < 1e-4, f"ratio={float(r):.4f}")
m.latest = [f32(2.0), f32(2.0)]           # -23.4 % -> kappen
r = m.get_averaged_size_ratio(f32(0.0))
check("T8 negativ auf -20 % gekappt", abs(float(r) + 20.0) < 1e-4, f"ratio={float(r):.4f}")

# --- T9: watchdog vs. explicit M406 ---------------------------------------------
print("\nT9: Watchdog respektiert explizites M406")
def t9_trip(model):
    for _ in range(8):
        model._fail("device_not_ready")
m = make_new()
t9_trip(m); m.correction_enabled = False; m.held_correction = False  # M406 waehrend tripped
m.handle_frame(build_frame(1.75, 1.75))
check("T9 Recovery aktiviert Korrektur NICHT nach M406", m.correction_enabled is False)
m = make_new()
m.correction_enabled = False; m.held_correction = False
t9_trip(m)                                 # Trip bei bereits aus
m.handle_frame(build_frame(1.75, 1.75))
check("T9 Trip bei bereits aus -> bleibt aus", m.correction_enabled is False)
m = make_new(); t9_trip(m)
m.consec_failures = 0; m.watchdog_tripped = False; m.correction_enabled = True  # M405-Reset
check("T9 M405-Reset stellt Zustand wieder her", m.correction_enabled is True and not m.watchdog_tripped)

# --- T10: retraction debt chains --------------------------------------------------
print("\nT10: Retraction-Debt-Ketten")
m = make_new()
seq = [((f32(5), 5.0, 0.0), (f32(-3), 5.0, 3.0), (f32(2), 5.0, 1.0), (f32(3), 7.0, 0.0))]
ok10 = True
for e, exp_pos, exp_debt in seq[0]:
    m.advance_e(e)
    ok10 = ok10 and float(m.pos) == exp_pos and float(m.debt) == exp_debt
check("T10 Kette A: 5 vor, 3 zurueck, 2+3 vor (nur +5 netto)", ok10,
      f"pos={float(m.pos)} debt={float(m.debt)}")
m = make_new(); m.advance_e(f32(10)); m.advance_e(f32(-10)); m.advance_e(f32(15))
check("T10 tiefer Retract voll zurueckgezahlt", float(m.pos) == 15.0 and float(m.debt) == 0.0)
m = make_new()
for e in (-2, -3, 1, -1, 7):
    m.advance_e(f32(e))
check("T10 Kette B: Schuld akkumuliert (pos +2)", float(m.pos) == 2.0 and float(m.debt) == 0.0)

# --- T11: queue integrity over long run -------------------------------------------
print("\nT11: Queue-Integritaet ueber 500 mm (Ring-Wrap, Ziel-Monotonie)")
m = make_new()
last_t = [f32(-1), f32(-1)]
mono = cap_ok = True
for step in range(500):
    m.advance_e(f32(1.0))
    m.handle_frame(build_frame(prof2(f32(m.pos + OFFSET1_MM)), prof2(f32(m.pos + OFFSET2_MM))))
    for ax in (0, 1):
        if m.size[ax] > m.cap: cap_ok = False
        if m.size[ax]:
            t11 = m.targets[ax][m.head[ax]]
            if float(t11) < float(last_t[ax]) - 1e-3: mono = False
            last_t[ax] = t11
for _ in range(300):
    m.advance_e(f32(1.0))
check("T11 Kapazitaet nie ueberschritten, Ziele streng monoton", mono and cap_ok)
check("T11 Queues nach 300mm Nachlauf leer", m.size[0] == 0 and m.size[1] == 0)

# --- T12: integral volume exactness ------------------------------------------------
print("\nT12: Lookahead-Integral vs. analytische Erwartung (lineare Rampe)")
m = make_new()
for step in range(200):
    m.advance_e(f32(1.0))
    a = f32(1.75 + 0.001 * (m.pos + OFFSET1_MM))
    m.handle_frame(build_frame(a, f32(1.75)))
r = m.get_averaged_size_ratio(f32(50.0))
# Ziel der Probe i ist (pos_beim_Enqueue + 140): Bins fuer [200,250) = Rampenwerte 200..249
bins = [1.75 + 0.001 * (200 + i) for i in range(50)]
avg_a = sum(bins) / 50
exp_ratio = (1.75 * 1.75 / (avg_a * 1.75)) * 100 - 100
check("T12 Integral = laengengewichtetes Mittel der Bins",
      abs(float(r) - exp_ratio) < 0.01, f"r={float(r):.4f} exp={exp_ratio:.4f}")

# --- T14: decode robustness --------------------------------------------------------
print("\nT14: Decode-Robustheit (ASCII, Muell, Version, CRC, Bereichsgrenzen)")
def raw_frame(da, db, version=PROTOCOL_VERSION):
    f = bytearray(list(da) + list(db) + [version, 0])
    f[11] = crc8(bytes(f[:11]))
    return bytes(f)
m = make_new(); m.advance_e(f32(1)); m.next_enqueue = f32(0)
r = m.handle_frame(raw_frame([ord('1'), ord('7'), ord('5'), ord('0'), ord('0')], [1, 7, 5, 0, 0]))
check("T14 ASCII-Ziffern toleriert", r[0] == 'ok' and m.size[0] == 1)
m = make_new(); m.advance_e(f32(1)); m.next_enqueue = f32(0)
r = m.handle_frame(raw_frame([0x0B, 7, 5, 0, 0], [1, 7, 5, 0, 0]))
check("T14 ungueltiges Byte -> decode_failed", r == ('fail', 'decode_failed'))
m = make_new()
r = m.handle_frame(raw_frame([1, 7, 5, 0, 0], [1, 7, 5, 0, 0], version=0x02))
check("T14 falsche Version -> bad_version", r == ('fail', 'bad_version'))
f = bytearray(raw_frame([1, 7, 5, 0, 0], [1, 7, 5, 0, 0])); f[3] ^= 1
m = make_new()
r = m.handle_frame(bytes(f))
check("T14 CRC-Bitflip -> crc_failed", r == ('fail', 'crc_failed'))
all_ok = True
for val, acc in (([0, 4, 9, 9, 9], False), ([0, 5, 0, 0, 0], True), ([3, 5, 0, 0, 0], True), ([3, 5, 0, 0, 1], False)):
    m = make_new(); m.advance_e(f32(1)); m.next_enqueue = f32(0)
    r = m.handle_frame(raw_frame(val, [1, 7, 5, 0, 0]))
    if (r[0] == 'ok') != acc:
        all_ok = False; check(f"T14 Bereich {val}", False, str(r)); break
if all_ok:
    check("T14 Bereichsgrenzen 4999/5000/35000/35001 korrekt", True)

# --- T16: planner chain (e_factor, mstep-Rundung) -----------------------------------
print("\nT16: Planner-Kette (apply_filament_width_sensor + e_msteps-Rundung)")
def planner_chain(de, ratio_pct, volumetric, flow_pct=100.0, nominal=1.75):
    corr = f32(f32(1) + f32(f32(0.01) * f32(ratio_pct)))
    if volumetric:
        base = f32(f32(f32(f32(f32(f32(3.14159265) * f32(0.25)) * nominal) * nominal)))
        mult = f32(corr / base)
    else:
        mult = corr
    e_factor = f32(f32(f32(flow_pct) * f32(0.01)) * mult)
    # Firmware (planner.cpp, seit Testdruck-Fix): e_msteps = |de*e_factor|+0.5,
    # im LOG mit Vorzeichen von de (Retraktion loggt negativ, nicht +3214 bei -3040)
    mag = int(abs(f32(f32(float(de)) * e_factor)) + f32(0.5))
    corr_steps = -mag if de < 0 else mag
    return float(e_factor), int(de), corr_steps
ef, raw, corr = planner_chain(260, 0.3, volumetric=False)
check("T16 Beispiel 260 Msteps x 1.003 -> 261", raw == 260 and corr == 261,
      f"e_factor={ef:.5f} {raw}->{corr}")
ef, raw, corr = planner_chain(1520, -5.4784, volumetric=False)
check("T16 1mm-Block mit -5.4784 % (1.80mm-Fall)", abs((corr - raw) / raw + 0.0548) < 0.001,
      f"{raw}->{corr}")
ef, raw, corr = planner_chain(50, 20.0, volumetric=False)
check("T16 Clamped-Input (+20 %) auf winzigem Block", abs(ef - 1.2) < 1e-5 and corr == 60,
      f"e_factor={ef:.5f} {raw}->{corr}")
ef, raw, corr = planner_chain(-3040, 6.0, volumetric=False)   # Abbruch-Retraktion vom Testdruck
check("T16 Retraction behaelt Vorzeichen (Testdruck-Fall -3040)",
      corr < 0 and abs(abs(corr) - 3222) <= 2,
      f"{raw}->{corr}")

# --- T17: long-run stability ---------------------------------------------------------
print("\nT17: Langzeitstabilitaet (10.000 Schritte, zufaellige Durchmesser)")
import random as _rand
_rand.seed(42)
m = make_new()
finite = True
for step in range(10000):
    m.advance_e(f32(1.0))
    a = f32(_rand.uniform(1.55, 1.95))
    b = f32(_rand.uniform(1.55, 1.95))
    m.handle_frame(build_frame(a, b))
    r = m.get_averaged_size_ratio(f32(1.0))
    if not (abs(float(r)) < 1e6):
        finite = False; break
check("T17 alle 10.000 Korrekturen endlich, keine Entartung", finite)
check("T17 Queue-Konsistenz nach 10.000 Schritten",
      m.size[0] <= m.cap and m.size[1] <= m.cap and float(m.pos) == 10000.0)

# --- T18: Kaltstart-Semantik (Warm-up-Phase) ----------------------------------------
# Nachvollzug des Testdrucks 2026-11-16: die Korrektur bleibt nach einem echten
# Kaltstart (leere Queues, pos=0) SOLANGE auf 0 % (ef=1.00), bis die erste
# Sample-Zielposition (pos+140 bzw. pos+213.8) erreicht ist — das Lookahead-
# Integral edge-holdet auf latest_axes, und latest_axes ist POP-basiert
# (Initialwert: nominal), NICHT der aktuelle Sensorframe. Erst ab pos>=140
# fliesst Achse-A-Historie, ab pos>=213.8 auch Achse B. Beim Testdruck war die
# Pipe durch die Laderoutine des Druckers (~140 mm getracktes E) vorbefuellt,
# deshalb waren die Uebergaenge schon im/fruehen Druck sichtbar.
print("\nT18: Kaltstart — Korrektur inaktiv bis erster Pop, dann Historie-basiert")
m = make_new()

def cold_step(a_mm, b_mm):
    m.advance_e(f32(1.0))
    m.handle_frame(build_frame(a_mm, b_mm, with_crc=True))
    return m.get_averaged_size_ratio(f32(1.0))

def ratio_of(a, b):
    return float(clamp_ratio(f32(f32(f32(NOMINAL_AREA / get_area_mm2(a, b)) * f32(100.0)) - f32(100.0))))

# Phase 1: pos 1..10, Sensor liefert 1.60/1.60. Keine Queue-Abdeckung im Block-
# Bereich -> Edge-Hold auf latest_axes (= nominal-Init, NICHT aktueller Frame)
# -> Korrektur 0 % (ef bleibt 1.00): der Drucker druckt unkorigiert weiter.
r_first = None
for _ in range(10):
    r = cold_step(f32(1.60), f32(1.60))
    if r_first is None: r_first = r
check("T18 Kaltstart: Korrektur 0 % (latest=nominial, kein Pop)",
      abs(float(r_first)) < 1e-6 and abs(float(r)) < 1e-6,
      f"erste={float(r_first):.4f}, letzte={float(r):.4f}")

# Phase 2: Pipe konsequent mit 1.60/1.60 fuellen, bis Achse-A-Ziele in den
# Blockbereich fallen (pos >= ~141): ab da A aus der Historie (1.60), B noch
# auf nominal-Init (1.75) -> Mischratio grob zwischen 0 % und dem A-Wert
while m.pos < f32(141.0):
    cold_step(f32(1.60), f32(1.60))
r = cold_step(f32(1.60), f32(1.60))
exp_mix = ratio_of(f32(1.60), f32(1.75))                  # ~ +9.4 %
check("T18 ab 141 mm: Achse A aus Historie (Mischwert, deutlich > 0)",
      float(r) > 5.0 and abs(float(r) - exp_mix) < 3.0,
      f"r={float(r):.2f}, erwartet ~{exp_mix:.2f}")

# Phase 3: bis B-Abdeckung (pos >= ~215), dann ist die Korrektur komplett
# historiebasiert; ein Sprung des AKTUELLEN Frames aendert die Korrektur
# NICHT sofort (latest_axes ist pop-basiert)
while m.pos < f32(215.0):
    cold_step(f32(1.60), f32(1.60))
r_hist = cold_step(f32(1.60), f32(1.60))
exp_full = ratio_of(f32(1.60), f32(1.60))                 # ~ +19.6 %
check("T18 ab 215 mm: vollstaendig historiebasiert (beide Pipes gefuellt)",
      abs(float(r_hist) - exp_full) < 2.0 and float(r_hist) > 15.0,
      f"r={float(r_hist):.2f}, erwartet ~{exp_full:.2f}")
r_after_jump = cold_step(f32(1.75), f32(1.75))            # aktueller Frame springt
check("T18 Frame-Sprung aendert Korrektur NICHT sofort (pop-basiertes latest)",
      abs(float(r_after_jump) - float(r_hist)) < 1.5,
      f"vor={float(r_hist):.2f}, nach={float(r_after_jump):.2f}")
check("T18 Clamp korrekt (Wert <= +20 %)", float(r_hist) <= 20.0 + 1e-6)

# --- T19: log capture semantics (z_mm, block_seq, cum_e_mm, flags, M407 T1) ------
print("\nT19: Log-Capture-Semantik (block_seq/Drops, cum_e, Travel-Flag T1)")

class LogModel:
    """Mirrors FilamentWidthSensor::capture_log_entry (filwidth.cpp, 10113+)."""
    def __init__(self, capacity=280):
        self.capacity = capacity
        self.start_logging()

    def start_logging(self):
        self.logging = True
        self.buf = []
        self.dropped = 0
        self.seq = 0
        self.cum_e = 0.0

    def capture(self, x, y, z, e_fac, msteps_raw, msteps_corr, de_mm, travel=False):
        if not self.logging:
            return
        self.seq += 1            # advances even on drop -> gap = drop signal
        self.cum_e += de_mm
        if len(self.buf) >= self.capacity:
            self.dropped += 1
            return
        self.buf.append((x, y, z, e_fac, msteps_raw, msteps_corr,
                         self.seq, self.cum_e, 1 if travel else 0))

lm = LogModel(capacity=4)
lm.capture(10, 20, 0.2, 1.0, 100, 100, 100.0 / 1520.0)
lm.capture(10, 30, 0.2, 1.1, 200, 220, 200.0 / 1520.0)
check("T19 seq monoton ab 1, cum_e in mm", lm.buf[0][6] == 1 and lm.buf[1][6] == 2
      and abs(lm.buf[1][7] - 300.0 / 1520.0) < 1e-9)

# Retraction: negative de_mm reduces cum_e, corrected msteps keep the sign
lm.capture(10, 30, 0.2, 1.0, -80, -80, -80.0 / 1520.0)
check("T19 Retraktion: cum_e sinkt, msteps_corr negativ",
      lm.buf[2][7] < lm.buf[1][7] and lm.buf[2][5] == -80)

# Travel block (M407 T1): de == 0 -> cum_e unchanged, flags bit 0 set
cum_before = lm.buf[-1][7]
lm.capture(50, 30, 0.2, 1.0, 0, 0, 0.0, travel=True)
check("T19 Travel: cum_e unveraendert, flags=1",
      lm.buf[3][7] == cum_before and lm.buf[3][8] == 1)

# Overflow: capacity 4 reached -> drops create seq gaps that survive the flush
lm.capture(60, 30, 0.2, 1.0, 50, 50, 50.0 / 1520.0)   # seq 5 -> dropped
lm.capture(60, 40, 0.4, 1.0, 60, 60, 60.0 / 1520.0)   # seq 6 -> dropped
lm.buf.clear()                                        # flush_log drained the buffer
lm.capture(60, 50, 0.4, 1.0, 70, 70, 70.0 / 1520.0)   # seq 7 -> buffered again
check("T19 Drops: seq-Sprung 4 -> 7 im CSV (2 verloren), dropped=2",
      lm.dropped == 2 and lm.buf[-1][6] == 7 and len(lm.buf) == 1)

# z_mm passes through per block (layer change visible in the log)
check("T19 z_mm je Block (neue Schicht 0.4 im Log)", lm.buf[-1][2] == 0.4)

# Fresh session (M407 L1) resets seq, cum_e and the drop counter
lm.start_logging()
lm.capture(0, 0, 0.2, 1.0, 10, 10, 10.0 / 1520.0)
check("T19 M407 L1-Reset: seq und cum_e starten neu",
      lm.buf[0][6] == 1 and abs(lm.buf[0][7] - 10.0 / 1520.0) < 1e-9 and lm.dropped == 0)

# --- Summary ------------------------------------------------------------------
print("\n" + "=" * 74)
failed = [r for r in results if r[1] == FAIL]
print(f"RESULT: {len(results) - len(failed)}/{len(results)} checks passed")
for name, status, detail in failed:
    print(f"  FAILED: {name} ({detail})")
sys.exit(1 if failed else 0)

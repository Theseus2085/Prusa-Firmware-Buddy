#!/usr/bin/env python3
"""
fwlog_graph.py - render graphs from captured FWLOG filament-sensor data

Input:  a *_fwlog.csv produced by fwlog_capture.py (columns time_ms, x_mm,
        y_mm, e_factor, msteps_raw, msteps_corr, axis_a_mm, axis_b_mm,
        ellipse_area_mm2). With --raw you can feed the raw log directly;
        FWLOG rows are filtered on the fly.

Output (PNG, 150 dpi, into --out-dir, default: beside the CSV):
  graph_diameters.png   axis A/B + equivalent diameter vs time, nominal line
  graph_correction.png  applied e_factor and correction % vs time
  graph_msteps.png      raw vs corrected E msteps vs time (quantization)
  graph_area.png        length-weighted ellipse area vs time, nominal area
  graph_xy_map.png      XY map of the print colored by applied correction

Requires: matplotlib + numpy  (pip install matplotlib numpy)
"""

import argparse
import csv
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

COLS = ["time_ms", "x_mm", "y_mm", "e_factor", "msteps_raw",
        "msteps_corr", "axis_a_mm", "axis_b_mm", "ellipse_area_mm2"]


def load_rows(path):
    """Read a FWLOG csv (or raw log with FWLOG, prefix) into a dict of columns."""
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("FWLOG,"):
                line = line[len("FWLOG,"):]
                if line.startswith("time_ms,"):
                    continue
                parts = line.split(",")
            elif line.startswith("time_ms,"):
                continue
            else:
                parts = line.split(",")
            if len(parts) != 9:
                continue
            try:
                rows.append([float(p) for p in parts])
            except ValueError:
                continue
    if not rows:
        bail(f"no FWLOG data rows found in {path}")
    data = np.array(rows, dtype=np.float64)
    data = data[np.isfinite(data).all(axis=1)]
    if not len(data):
        bail("all rows contained non-finite values")
    return {name: data[:, i] for i, name in enumerate(COLS)}


def bail(msg):
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"[fwlog] wrote {path}")


def graph_diameters(d, out, nominal):
    t = d["time_ms"] / 1000.0
    a, b = d["axis_a_mm"], d["axis_b_mm"]
    deq = np.sqrt(a * b)
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(t, a, lw=0.8, alpha=0.8, label="Achse 1")
    ax.plot(t, b, lw=0.8, alpha=0.8, label="Achse 2")
    ax.plot(t, deq, lw=1.2, color="k", label="Ersatzdurchmesser $\\sqrt{a\\cdot b}$")
    ax.axhline(nominal, color="r", ls="--", lw=1, label=f"Nominal {nominal:.2f} mm")
    ax.set_xlabel("Zeit [s]")
    ax.set_ylabel("Durchmesser [mm]")
    ax.set_title("Gemessene Filamentdurchmesser an der Düse")
    ax.legend(loc="best")
    ax.grid(alpha=0.3)
    save(fig, os.path.join(out, "graph_diameters.png"))


def graph_correction(d, out):
    t = d["time_ms"] / 1000.0
    ef = d["e_factor"]
    pct = (ef - 1.0) * 100.0
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.step(t, ef, where="post", lw=1.2, color="tab:blue", label="e_factor (Volumenkorrektur)")
    ax.axhline(1.0, color="k", lw=0.8, ls=":")
    ax.set_xlabel("Zeit [s]")
    ax.set_ylabel("e_factor", color="tab:blue")
    ax2 = ax.twinx()
    ax2.step(t, pct, where="post", lw=0.9, alpha=0.55, color="tab:orange",
             label="Korrektur [%]")
    ax2.set_ylabel("Korrektur [%]", color="tab:orange")
    n_off = int((ef == 1.0).sum())
    ax.set_title(f"Angewandte Extrusionskorrektur je Planner-Block "
                 f"(e_factor==1.0 bei {n_off}/{len(ef)} Blöcken)")
    ax.grid(alpha=0.3)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="best")
    save(fig, os.path.join(out, "graph_correction.png"))


def graph_msteps(d, out):
    t = d["time_ms"] / 1000.0
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.step(t, d["msteps_raw"], where="post", lw=1.0, alpha=0.8,
            label="msteps_raw (de)")
    ax.step(t, d["msteps_corr"], where="post", lw=0.8, alpha=0.8,
            label="msteps_corr (de * e_factor)")
    ax.set_xlabel("Zeit [s]")
    ax.set_ylabel("E-Microsteps (1520/mm)")
    ax.set_title("Roh- vs. korrigierte E-Microsteps je Block (Quantisierung sichtbar)")
    ax.legend(loc="best")
    ax.grid(alpha=0.3)
    save(fig, os.path.join(out, "graph_msteps.png"))


def graph_area(d, out, nominal):
    t = d["time_ms"] / 1000.0
    nom_area = math.pi / 4.0 * nominal * nominal
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(t, d["ellipse_area_mm2"], lw=1.0, color="tab:green",
            label="längengewichtete mittlere Querschnittsfläche")
    ax.axhline(nom_area, color="r", ls="--", lw=1,
               label=f"Nominalfläche {nom_area:.3f} mm²")
    ax.set_xlabel("Zeit [s]")
    ax.set_ylabel("Fläche [mm²]")
    ax.set_title("Berechnete Ellipsen-Querschnittsfläche (Lookahead-Integral)")
    ax.legend(loc="best")
    ax.grid(alpha=0.3)
    save(fig, os.path.join(out, "graph_area.png"))


def graph_xy_map(d, out):
    ef = d["e_factor"]
    pct = (ef - 1.0) * 100.0
    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    sc = ax.scatter(d["x_mm"], d["y_mm"], c=pct, cmap="coolwarm",
                    vmin=-max(abs(pct).max(), 1), vmax=max(abs(pct).max(), 1),
                    s=6, lw=0)
    fig.colorbar(sc, ax=ax, label="Korrektur [%] (rot = weniger, blau = mehr E)")
    ax.set_aspect("equal")
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_title("Korrektur über der Druckfläche (Block-Zielpunkte)")
    ax.grid(alpha=0.3)
    save(fig, os.path.join(out, "graph_xy_map.png"))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="*_fwlog.csv from fwlog_capture.py "
                    "(or a raw log, detected via FWLOG, prefix)")
    ap.add_argument("--out-dir", help="output directory (default: beside the CSV)")
    ap.add_argument("--nominal", type=float, default=1.75,
                    help="nominal filament diameter for reference lines (default 1.75)")
    args = ap.parse_args()

    if not os.path.isfile(args.csv):
        bail(f"file not found: {args.csv}")
    out = args.out_dir or os.path.dirname(os.path.abspath(args.csv))
    os.makedirs(out, exist_ok=True)

    d = load_rows(args.csv)
    n = len(d["time_ms"])
    dur = (d["time_ms"][-1] - d["time_ms"][0]) / 1000.0
    print(f"[fwlog] {n} entries, span {dur:.1f}s "
          f"(x {d['x_mm'].min():.0f}..{d['x_mm'].max():.0f} mm, "
          f"y {d['y_mm'].min():.0f}..{d['y_mm'].max():.0f} mm)")

    graph_diameters(d, out, args.nominal)
    graph_correction(d, out)
    graph_msteps(d, out)
    graph_area(d, out, args.nominal)
    graph_xy_map(d, out)
    print(f"[fwlog] {5} graphs -> {out}")


if __name__ == "__main__":
    main()

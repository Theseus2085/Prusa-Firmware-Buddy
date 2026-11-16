#!/usr/bin/env python3
"""Inspect a 3MF build plate: per-object bounding boxes + plate layout.
Usage: python inspect_3mf.py <file.3mf> [file2.3mf ...]"""
import sys, zipfile, re
import xml.etree.ElementTree as ET

NS = "{http://schemas.microsoft.com/3dmanufacturing/core/2015/02}"

def fmt(v): return f"{v:7.1f}"

for path in sys.argv[1:]:
    print(f"\n=== {path} ===")
    try:
        z = zipfile.ZipFile(path)
    except Exception as e:
        print("  not a zip/3mf:", e)
        continue
    names = z.namelist()
    model_name = "3D/3dmodel.model"
    if model_name not in names:
        print("  contents:", [n for n in names if not n.endswith("/")][:15])
        continue
    root = ET.fromstring(z.read(model_name))

    # objects: id -> mesh bbox (model space of the object)
    objs = {}
    for obj in root.iter(NS + "object"):
        oid = obj.get("id")
        verts = []
        for mesh in obj.iter(NS + "mesh"):
            for v in mesh.iter(NS + "vertex"):
                verts.append((float(v.get("x", 0)), float(v.get("y", 0)), float(v.get("z", 0))))
        if verts:
            xs, ys, zs = zip(*verts)
            objs[oid] = (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs), len(verts))
    comps = {}
    for comp in root.iter(NS + "component"):
        comps.setdefault(comp.get("objectid", "?"), []).append(
            comp.get("transform", ""))

    print(f"  objects with mesh: {len(objs)}   (components refs: {len(comps)})")

    def apply_transform(t, bbox):
        """Apply 3MF 'm00 m01 m02 m10 ... m32 m33' (12 numbers, row-major 4x3)
        to bbox corners. Only affine - sample all 8 corners."""
        if not t:
            return bbox
        m = [float(x) for x in t.split()]
        if len(m) != 12:
            return bbox
        M = [m[0:3], m[3:6], m[6:9]]
        T = m[9:12]
        x0, x1, y0, y1, z0, z1 = bbox
        pts = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
        out = []
        for p in pts:
            out.append(tuple(sum(M[r][c] * p[c] for c in range(3)) + T[r]
                             for r in range(3)))
        xs, ys, zs = zip(*out)
        return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

    items = []
    for item in root.iter(NS + "item"):
        oid = item.get("objectid")
        if oid not in objs:
            continue
        base = objs[oid][:6]
        bb = apply_transform(item.get("transform", ""), base)
        items.append((oid, bb, objs[oid][6]))

    # expand composite objects one level (PrusaSlicer often nests components)
    expanded = []
    for oid, bb, nv in items:
        if oid in comps:
            for cref, ct in comps[oid]:
                if cref in objs:
                    child = apply_transform(ct or "", objs[cref][:6])
                    child = apply_transform(item_tf := "", child)  # already merged? keep simple
                    # merge item transform too
                    it = item_tf_from = None
                    expanded.append((cref, child, objs[cref][6]))
        else:
            expanded.append((oid, bb, nv))

    print(f"  build items: {len(items)}")
    print(f"  {'id':>3} {'X0':>7} {'X1':>7} {'Y0':>7} {'Y1':>7} {'Z0':>5} {'Z1':>5}  {'verts':>7}  {'WxH':>12}")
    for oid, bb, nv in items:
        x0, x1, y0, y1, z0, z1 = bb
        print(f"  {oid:>3} {fmt(x0)} {fmt(x1)} {fmt(y0)} {fmt(y1)} {z0:5.1f} {z1:5.1f}  {nv:7d}  {x1-x0:5.1f}x{y1-y0:5.1f}x{z1-z0:4.1f}")
    if items:
        X0 = min(b[1] for _, b, _ in items); X1 = max(b[2] for _, b, _ in items)
        Y0 = min(b[3] for _, b, _ in items); Y1 = max(b[4] for _, b, _ in items)
        print(f"  TOTAL plate extent: X {X0:.1f}..{X1:.1f}  Y {Y0:.1f}..{Y1:.1f}")

    # config metadata if present
    for cfg in ("Metadata/Slic3r_PE.config", "Metadata/Prusa_Slicer.config"):
        if cfg in names:
            txt = z.read(cfg).decode("utf-8", errors="replace")
            interesting = [l for l in txt.splitlines()
                           if re.search(r"^(print_settings|filament_settings|printer_settings|printer_model|nozzle_diameter|complete_objects|arc_fitting|external_perimeter_speed|infill_speed|perimeter_speed|layer_height|filament_type)\b", l)]
            print(f"  --- {cfg} (selected keys) ---")
            for l in interesting[:25]:
                print("   ", l.strip())

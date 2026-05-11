#!/usr/bin/env python3
"""
segment_gcode.py — G-Code Segment Splitter for Filament Width Sensor Optimization

Prevents long linear moves (G0/G1) from monopolizing the Marlin planner block
buffer. A single 200mm infill line becomes one planner block that executes for
~2s at 100mm/s — during which no new filament width sensor correction can be
applied. This script splits such moves into shorter sub-segments.

Segmentation strategy:
  - PRIMARY:  XY head travel distance drives the split (--max-segment-mm).
  - GUARD:    For extrusion moves, the segment count is reduced if each sub-
              segment would consume less than --min-e-per-segment-mm of filament.
              This prevents creating segments below the sensor's spatial
              resolution (FILWIDTH_SENSOR_QUEUE_BIN_MM = 1mm).

Sensor logging injection:
  When --sensor-lines N is given (default 8), the script automatically:
    - Injects M407 L1 before the 1st detected print line to start logging.
      (The sensor itself is assumed to already be enabled at print start.)
    - Injects M407 L0 + M407 D + M406 after the Nth print line.
  A "print line" is any G1 move that has both XY travel and extrusion (E).

Usage as PrusaSlicer post-processor:
    "C:\\path\\to\\python.exe" "C:\\path\\to\\segment_gcode.py" --max-segment-mm 25

PrusaSlicer appends the G-code file path as the final argument automatically.

Author:  Generated for Prusa-Firmware-Buddy filament width sensor project
License: GPLv3 (matching firmware)
"""

import sys
import re
import math
import os
import argparse
import time

# ---------------------------------------------------------------------------
# Version
# ---------------------------------------------------------------------------
__version__ = "1.2.0"

# ---------------------------------------------------------------------------
# G-code parameter regex — matches e.g. "X123.456", "E-0.800", "F3600"
# ---------------------------------------------------------------------------
_PARAM_RE = re.compile(r'([XYZEF])([-+]?\d*\.?\d+)')

# G-command regex — matches G0 or G1 (case-insensitive, with optional spaces)
_GCMD_RE = re.compile(r'^[Gg]\s*([01])\b')


def parse_gcode_params(line: str) -> dict:
    """Extract G-code parameters from a line into a dict.

    Returns e.g. {'X': 100.0, 'Y': 50.0, 'E': 1.234, 'F': 3600.0}
    Only parameters present in the line are included.
    """
    params = {}
    for match in _PARAM_RE.finditer(line):
        params[match.group(1)] = float(match.group(2))
    return params


def format_coord(value: float, decimals: int = 5) -> str:
    """Format a coordinate value, stripping unnecessary trailing zeros.

    Uses enough decimal places to avoid cumulative rounding errors on
    the sub-segment interpolation while keeping the output readable.
    """
    formatted = f"{value:.{decimals}f}"
    # Strip trailing zeros but keep at least one decimal place
    if '.' in formatted:
        formatted = formatted.rstrip('0').rstrip('.')
        # Ensure at least one decimal for readability if originally fractional
        if '.' not in formatted:
            formatted += '.0'
    return formatted


def build_gcode_line(cmd: str,
                     params: dict,
                     include_f: bool = False,
                     comment: str = "") -> str:
    """Reconstruct a G-code line from parsed components.

    Args:
        cmd: The G command string, e.g. "G1"
        params: Dict of parameter values to emit
        include_f: Whether to include the F parameter
        comment: Optional inline comment to append
    """
    parts = [cmd]
    for axis in ('X', 'Y', 'Z', 'E'):
        if axis in params:
            parts.append(f"{axis}{format_coord(params[axis])}")
    if include_f and 'F' in params:
        parts.append(f"F{format_coord(params['F'], 1)}")
    line = ' '.join(parts)
    if comment:
        line += f" ; {comment}"
    return line


def calculate_xy_distance(dx: float, dy: float) -> float:
    """Euclidean distance in the XY plane."""
    return math.sqrt(dx * dx + dy * dy)


def compute_segment_count(xy_dist: float, abs_de: float, max_seg_mm: float,
                          min_seg_mm: float, min_e_per_seg: float) -> int:
    """Determine the optimal number of sub-segments for a move.

    Primary criterion: XY distance / max_seg_mm  → how many segments we'd want.
    Guard #1 (XY floor): Each segment must be >= min_seg_mm in XY.
    Guard #2 (E floor):  For extrusion moves, each segment must consume
                         >= min_e_per_seg mm of filament so the sensor has
                         at least one queue bin of data per block.

    Args:
        xy_dist:       XY head travel distance of the full move (mm)
        abs_de:        Absolute E delta of the full move (mm of filament)
        max_seg_mm:    Target maximum XY length per segment
        min_seg_mm:    Minimum XY length per segment (safety floor)
        min_e_per_seg: Minimum filament consumption per segment (mm)

    Returns:
        Number of segments (1 = don't split).
    """
    if xy_dist <= max_seg_mm:
        return 1

    # Start with the ideal count from XY distance
    n = math.ceil(xy_dist / max_seg_mm)

    # Guard #1: XY segments must not be too short
    while n > 1 and (xy_dist / n) < min_seg_mm:
        n -= 1

    # Guard #2: E distance per segment must meet the sensor minimum.
    # Only applies when there is actual extrusion (abs_de > 0).
    # Travel moves (abs_de == 0) skip this check — they have no sensor
    # interaction and should still be split to free up the planner.
    if abs_de > 0.0 and min_e_per_seg > 0.0:
        while n > 1 and (abs_de / n) < min_e_per_seg:
            n -= 1

    return n


def segment_move(current_pos: dict, params: dict, cmd: str, max_seg_mm: float,
                 min_seg_mm: float, min_e_per_seg: float) -> list:
    """Split a single G0/G1 move into sub-segments if it exceeds thresholds.

    Args:
        current_pos:   Current head position {'X': ..., 'Y': ..., 'Z': ..., 'E': ...}
        params:        Parsed parameters of this move
        cmd:           "G0" or "G1"
        max_seg_mm:    Target maximum XY segment length (mm)
        min_seg_mm:    Minimum XY segment length (mm)
        min_e_per_seg: Minimum filament per segment (mm)

    Returns:
        List of G-code line strings, or None if the move should be emitted as-is.
    """
    # Determine start and end positions for axes present in this move
    start_x = current_pos.get('X', 0.0)
    start_y = current_pos.get('Y', 0.0)
    start_z = current_pos.get('Z', 0.0)
    start_e = current_pos.get('E', 0.0)

    end_x = params.get('X', start_x)
    end_y = params.get('Y', start_y)
    end_z = params.get('Z', start_z)
    end_e = params.get('E', start_e)

    dx = end_x - start_x
    dy = end_y - start_y
    dz = end_z - start_z
    de = end_e - start_e

    # Calculate the move distance in XY (primary segmentation axis)
    xy_dist = calculate_xy_distance(dx, dy)

    # --- Skip moves that should never be segmented ---

    # Don't segment pure Z moves (layer changes) or pure E moves (retraction)
    has_xy = ('X' in params or 'Y' in params)
    has_z_only = ('Z' in params and not has_xy and 'E' not in params)
    has_e_only = ('E' in params and not has_xy and 'Z' not in params)

    if has_z_only or has_e_only:
        return None

    # For moves with a Z component but no/negligible XY, use XYZ distance
    # so that long ramp moves are still caught
    if xy_dist < 0.01 and abs(dz) > 0.01:
        xy_dist = math.sqrt(dx * dx + dy * dy + dz * dz)

    # --- Compute segment count with dual criteria ---
    abs_de = abs(de)
    n_segments = compute_segment_count(xy_dist, abs_de, max_seg_mm, min_seg_mm,
                                       min_e_per_seg)

    if n_segments <= 1:
        return None

    # --- Perform interpolation ---
    lines = []
    has_x = 'X' in params
    has_y = 'Y' in params
    has_z = 'Z' in params
    has_e = 'E' in params
    has_f = 'F' in params

    for i in range(1, n_segments + 1):
        t = i / n_segments  # Interpolation parameter [0..1]
        seg_params = {}

        if has_x:
            seg_params['X'] = start_x + dx * t
        if has_y:
            seg_params['Y'] = start_y + dy * t
        if has_z:
            seg_params['Z'] = start_z + dz * t
        if has_e:
            seg_params['E'] = start_e + de * t
        if has_f:
            seg_params['F'] = params['F']

        # Only include F on the first sub-segment (it's modal in G-code)
        include_f = (i == 1 and has_f)

        line = build_gcode_line(cmd, seg_params, include_f=include_f)
        lines.append(line)

    return lines


def is_print_line(params: dict) -> bool:
    """Return True if a G1 move has both XY travel and extrusion."""
    has_xy = 'X' in params or 'Y' in params
    has_e = 'E' in params
    return has_xy and has_e


def process_gcode(input_lines: list,
                  max_seg_mm: float,
                  min_seg_mm: float,
                  min_e_per_seg: float,
                  sensor_lines: int = 0) -> tuple:
    """Process all G-code lines, segmenting long moves.

    Args:
        input_lines:   List of raw G-code line strings
        max_seg_mm:    Target maximum XY segment length
        min_seg_mm:    Minimum XY segment length
        min_e_per_seg: Minimum filament per segment (mm)
        sensor_lines:  Number of print lines to wrap with M405/M407 sensor
                       logging injection. 0 = disabled.

    Returns:
        Tuple of (output_lines, stats_dict)
    """
    output_lines = []
    current_pos = {'X': 0.0, 'Y': 0.0, 'Z': 0.0, 'E': 0.0}

    # Track whether we're in absolute or relative mode
    absolute_xyz = True
    absolute_e = True

    # Sensor injection state
    print_line_count = 0  # number of print lines seen so far
    logging_started = False  # True after M407 L1 has been injected
    sensor_stopped = False  # True after M406 / M407 L0 D have been injected

    stats = {
        'original_moves': 0,
        'segmented_moves': 0,
        'segments_created': 0,
        'total_output_moves': 0,
        'longest_original_mm': 0.0,
        'skipped_by_e_guard': 0,
    }

    for line in input_lines:
        stripped = line.strip()

        # Pass through empty lines and pure comments
        if not stripped or stripped.startswith(';'):
            output_lines.append(line)
            continue

        # Detect coordinate mode changes
        if stripped.startswith('G90'):
            absolute_xyz = True
            absolute_e = True
            output_lines.append(line)
            continue
        elif stripped.startswith('G91'):
            absolute_xyz = False
            absolute_e = False
            output_lines.append(line)
            continue
        elif stripped.startswith('M82'):
            absolute_e = True
            output_lines.append(line)
            continue
        elif stripped.startswith('M83'):
            absolute_e = False
            output_lines.append(line)
            continue

        # Only process G0/G1 commands
        gcmd_match = _GCMD_RE.match(stripped)
        if not gcmd_match:
            output_lines.append(line)
            continue

        cmd_num = gcmd_match.group(1)
        cmd = f"G{cmd_num}"
        params = parse_gcode_params(stripped)

        if not params:
            output_lines.append(line)
            continue

        stats['original_moves'] += 1

        # --- Handle relative mode ---
        if not absolute_xyz or not absolute_e:
            rel_dx = params.get('X', 0.0)
            rel_dy = params.get('Y', 0.0)
            rel_dz = params.get('Z', 0.0)
            rel_de = params.get('E', 0.0)

            xy_dist = calculate_xy_distance(rel_dx, rel_dy)
            if xy_dist < 0.01 and abs(rel_dz) > 0.01:
                xy_dist = math.sqrt(rel_dx**2 + rel_dy**2 + rel_dz**2)

            stats['longest_original_mm'] = max(stats['longest_original_mm'],
                                               xy_dist)

            has_xy = ('X' in params or 'Y' in params)
            has_z_only = ('Z' in params and not has_xy and 'E' not in params)
            has_e_only = ('E' in params and not has_xy and 'Z' not in params)

            if has_z_only or has_e_only:
                output_lines.append(line)
                stats['total_output_moves'] += 1
                current_pos['X'] += rel_dx
                current_pos['Y'] += rel_dy
                current_pos['Z'] += rel_dz
                current_pos['E'] += rel_de
                continue

            abs_de = abs(rel_de)
            n_segments = compute_segment_count(xy_dist, abs_de, max_seg_mm,
                                               min_seg_mm, min_e_per_seg)

            if n_segments <= 1:
                output_lines.append(line)
                stats['total_output_moves'] += 1
                current_pos['X'] += rel_dx
                current_pos['Y'] += rel_dy
                current_pos['Z'] += rel_dz
                current_pos['E'] += rel_de
                continue

            has_f = 'F' in params
            stats['segmented_moves'] += 1
            stats['segments_created'] += n_segments

            for i in range(1, n_segments + 1):
                seg_params = {}
                if 'X' in params:
                    seg_params['X'] = rel_dx / n_segments
                if 'Y' in params:
                    seg_params['Y'] = rel_dy / n_segments
                if 'Z' in params:
                    seg_params['Z'] = rel_dz / n_segments
                if 'E' in params:
                    seg_params['E'] = rel_de / n_segments
                if has_f:
                    seg_params['F'] = params['F']

                include_f = (i == 1 and has_f)
                seg_line = build_gcode_line(cmd,
                                            seg_params,
                                            include_f=include_f)
                output_lines.append(seg_line + '\n')
                stats['total_output_moves'] += 1

            current_pos['X'] += rel_dx
            current_pos['Y'] += rel_dy
            current_pos['Z'] += rel_dz
            current_pos['E'] += rel_de
            continue

        # --- Absolute mode (the common case for PrusaSlicer) ---

        # Calculate move distance for statistics
        dx = params.get('X', current_pos['X']) - current_pos['X']
        dy = params.get('Y', current_pos['Y']) - current_pos['Y']
        xy_dist = calculate_xy_distance(dx, dy)
        dz = params.get('Z', current_pos['Z']) - current_pos['Z']
        if xy_dist < 0.01 and abs(dz) > 0.01:
            xy_dist = math.sqrt(dx**2 + dy**2 + dz**2)
        stats['longest_original_mm'] = max(stats['longest_original_mm'],
                                           xy_dist)

        # --- Sensor injection (absolute mode only) ---
        # Determine if this move qualifies as a "print line"
        this_is_print_line = (sensor_lines > 0 and cmd == 'G1'
                              and is_print_line(params) and not sensor_stopped)

        if this_is_print_line:
            if not logging_started:
                # Inject logging-start before the very first print line
                output_lines.append("; --- filwidth logging start ---\n")
                output_lines.append("M407 L1\n")  # start logging
                logging_started = True

        # Attempt segmentation
        result = segment_move(current_pos, params, cmd, max_seg_mm, min_seg_mm,
                              min_e_per_seg)

        if result is None:
            # Move is short enough or shouldn't be segmented
            output_lines.append(line)
            stats['total_output_moves'] += 1
        else:
            stats['segmented_moves'] += 1
            stats['segments_created'] += len(result)
            for seg_line in result:
                output_lines.append(seg_line + '\n')
            stats['total_output_moves'] += len(result)

        if this_is_print_line:
            print_line_count += 1
            if print_line_count >= sensor_lines and not sensor_stopped:
                # Inject logging-stop + dump + sensor-off after the Nth line
                output_lines.append(
                    "; --- filwidth logging stop + dump + sensor OFF ---\n")
                output_lines.append("M407 L0\n")  # stop logging
                output_lines.append("M407 D\n")  # dump log over serial
                output_lines.append("M406\n")  # disable filament width sensor
                sensor_stopped = True

        # Update current position with whatever this move sets
        if 'X' in params:
            current_pos['X'] = params['X']
        if 'Y' in params:
            current_pos['Y'] = params['Y']
        if 'Z' in params:
            current_pos['Z'] = params['Z']
        if 'E' in params:
            current_pos['E'] = params['E']

    return output_lines, stats


def build_stats_header(stats: dict, max_seg_mm: float, min_seg_mm: float,
                       min_e_per_seg: float, elapsed_s: float,
                       original_size: int, new_size: int) -> list:
    """Build the statistics comment block to inject at the top of the file."""
    size_delta_pct = ((new_size - original_size) / original_size *
                      100 if original_size > 0 else 0)
    lines = [
        f"; === FILWIDTH SEGMENTER v{__version__} ===\n",
        f"; Max XY segment: {max_seg_mm} mm (head travel)\n",
        f"; Min XY segment: {min_seg_mm} mm (floor)\n",
        f"; Min E per segment: {min_e_per_seg} mm (filament guard)\n",
        f"; Original G0/G1 moves: {stats['original_moves']}\n",
        f"; Moves segmented: {stats['segmented_moves']}\n",
        f"; Sub-segments created: {stats['segments_created']}\n",
        f"; Total output moves: {stats['total_output_moves']}\n",
        f"; Longest original move: {stats['longest_original_mm']:.1f} mm\n",
        f"; File size delta: {size_delta_pct:+.1f}%\n",
        f"; Processing time: {elapsed_s:.2f}s\n",
        f"; ==============================\n",
    ]
    return lines


def find_header_insert_position(lines: list) -> int:
    """Find the best position to insert the stats header.

    Inserts after any existing file-level comments/headers at the top
    (slicer version info, thumbnails, etc.) but before the first real command.
    """
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped and not stripped.startswith(';'):
            return i
        if i > 50:
            return i
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Segment long G-code moves for filament width sensor "
        "optimization. Designed as a PrusaSlicer post-processor.\n\n"
        "Splits based on XY head travel distance to prevent planner "
        "block monopoly, with a filament-distance guard to avoid "
        "creating sub-segments below the sensor's spatial resolution.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="PrusaSlicer appends the G-code file path as the last "
        "positional argument automatically.\n\n"
        "Typical filament consumption per mm of XY travel:\n"
        "  0.2mm layer, 0.45mm width, 1.75mm filament → ~0.037 mm E / mm XY\n"
        "  0.3mm layer, 0.45mm width, 1.75mm filament → ~0.056 mm E / mm XY\n\n"
        "At --max-segment-mm 25:\n"
        "  0.2mm layer → ~0.94 mm E/segment (≈1 sensor bin)\n"
        "  0.3mm layer → ~1.40 mm E/segment (>1 sensor bin)\n")
    parser.add_argument(
        'gcode_file',
        help="Path to the G-code file to process (modified in-place)")
    parser.add_argument(
        '--max-segment-mm',
        type=float,
        default=25.0,
        help="Maximum XY head travel per segment in mm. Drives the split. "
        "At 100mm/s, a 25mm segment executes in 0.25s. (default: 25.0)")
    parser.add_argument(
        '--min-segment-mm',
        type=float,
        default=5.0,
        help="Minimum XY segment length in mm — prevents tiny fragments. "
        "(default: 5.0)")
    parser.add_argument(
        '--min-e-per-segment-mm',
        type=float,
        default=1.0,
        help="Minimum filament consumption (E distance) per segment in mm. "
        "If splitting would create segments below this, fewer segments "
        "are created. Matches FILWIDTH_SENSOR_QUEUE_BIN_MM. (default: 1.0)")
    parser.add_argument(
        '--sensor-lines',
        type=int,
        default=8,
        help="Number of extrusion print lines to wrap with M407 sensor "
        "logging. 'M407 L1' is injected before line 1 (sensor is assumed "
        "already on), and 'M407 L0' + 'M407 D' + M406 after line N. "
        "Set to 0 to disable. (default: 8)")
    parser.add_argument('--version',
                        action='version',
                        version=f'%(prog)s {__version__}')

    args = parser.parse_args()

    gcode_path = args.gcode_file
    max_seg = args.max_segment_mm
    min_seg = args.min_segment_mm
    min_e = args.min_e_per_segment_mm
    sensor_lines = args.sensor_lines

    # Validate arguments
    if max_seg < 1.0:
        print(f"ERROR: --max-segment-mm must be >= 1.0, got {max_seg}",
              file=sys.stderr)
        sys.exit(1)
    if min_seg < 0.5:
        print(f"ERROR: --min-segment-mm must be >= 0.5, got {min_seg}",
              file=sys.stderr)
        sys.exit(1)
    if min_seg >= max_seg:
        print(
            f"ERROR: --min-segment-mm ({min_seg}) must be < "
            f"--max-segment-mm ({max_seg})",
            file=sys.stderr)
        sys.exit(1)
    if min_e < 0.0:
        print(f"ERROR: --min-e-per-segment-mm must be >= 0.0, got {min_e}",
              file=sys.stderr)
        sys.exit(1)

    if not os.path.isfile(gcode_path):
        print(f"ERROR: File not found: {gcode_path}", file=sys.stderr)
        sys.exit(1)

    # Read the input file
    print(f"[segment_gcode] Reading: {gcode_path}")
    with open(gcode_path, 'r', encoding='utf-8', errors='replace') as f:
        input_lines = f.readlines()

    original_size = os.path.getsize(gcode_path)

    # Process
    if sensor_lines > 0:
        print(f"[segment_gcode] Sensor injection: M405/M407 logging wrapping "
              f"first {sensor_lines} print line(s).")
    print(f"[segment_gcode] Processing with max_xy={max_seg}mm, "
          f"min_xy={min_seg}mm, min_e={min_e}mm ...")
    t_start = time.monotonic()
    output_lines, stats = process_gcode(input_lines, max_seg, min_seg, min_e,
                                        sensor_lines)
    t_elapsed = time.monotonic() - t_start

    # Calculate new file size (approximate, for the header)
    new_content = ''.join(output_lines)
    new_size = len(new_content.encode('utf-8'))

    # Build and insert statistics header
    header_lines = build_stats_header(stats, max_seg, min_seg, min_e,
                                      t_elapsed, original_size, new_size)
    insert_pos = find_header_insert_position(output_lines)
    for i, hline in enumerate(header_lines):
        output_lines.insert(insert_pos + i, hline)

    # Write output back in-place
    with open(gcode_path, 'w', encoding='utf-8', newline='\n') as f:
        f.writelines(output_lines)

    final_size = os.path.getsize(gcode_path)
    delta_pct = ((final_size - original_size) / original_size *
                 100 if original_size > 0 else 0)

    print(f"[segment_gcode] Done in {t_elapsed:.2f}s")
    print(f"[segment_gcode] Moves: {stats['original_moves']} -> "
          f"{stats['total_output_moves']} "
          f"(+{stats['segments_created'] - stats['segmented_moves']} lines)")
    print(f"[segment_gcode] Longest original move: "
          f"{stats['longest_original_mm']:.1f}mm")
    print(f"[segment_gcode] File size: {original_size} -> {final_size} bytes "
          f"({delta_pct:+.1f}%)")


if __name__ == '__main__':
    main()

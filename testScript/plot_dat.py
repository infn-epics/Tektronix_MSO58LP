#!/usr/bin/env python3
"""
plot_dat.py - Plot waveforms saved by the MSO58LP IOC store subsystem.

Supported file formats (produced by tekStoreWaveform.c):
    1) Legacy DAT:
       Line 1 : "# key=value key=value ..." metadata header
       Lines 2..N : one acquisition per line, comma-separated voltage samples.

    2) Tek-style CSV:
       Metadata preamble (Model/Label/Sample Interval/...)
       Followed by one or more acquisition blocks:
           Labels,<channel>
           TIME,<channel>
           <time_s>,<voltage_v>
           ...

Usage:
    python3 plot_dat.py FILE [--max N] [--mean] [--save out.png]
                       [--no-show] [--time]

Options:
    --max N     plot only the first N acquisitions (default: all)
    --mean      overlay the mean trace across all acquisitions
    --time      use absolute time on x-axis (xinc * sample_index +
                first_sample_time_s) instead of sample index
    --save P    save figure to path P
    --no-show   do not open the interactive window
"""
import argparse
import os
import sys

import numpy as np


def parse_header(line: str) -> dict:
    line = line.lstrip("# ").strip()
    meta = {}
    for tok in line.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            try:
                meta[k] = float(v) if any(c in v for c in ".eE") else int(v)
            except ValueError:
                meta[k] = v
    return meta


def load_dat(path: str):
    with open(path, "r") as f:
        header_line = f.readline()
    if not header_line.startswith("#"):
        raise ValueError(f"{path}: missing '#' metadata header")
    meta = parse_header(header_line)
    data = np.loadtxt(path, delimiter=",", comments="#")
    if data.ndim == 1:
        data = data[np.newaxis, :]
    return meta, data


def _to_float(s: str, default=0.0) -> float:
    try:
        return float(s)
    except Exception:
        return default


def load_tek_csv(path: str):
    with open(path, "r") as f:
        lines = [ln.rstrip("\n") for ln in f]

    if not lines or not lines[0].startswith("Model,"):
        raise ValueError(f"{path}: not a Tek-style CSV file")

    meta = {}
    idx = 0
    while idx < len(lines):
        raw = lines[idx].strip()
        if raw == "":
            idx += 1
            break
        parts = [p.strip() for p in lines[idx].split(",", 1)]
        key = parts[0]
        val = parts[1] if len(parts) > 1 else ""
        meta[key] = val
        idx += 1

    channel = meta.get("Label", "?")
    xinc = _to_float(meta.get("Sample Interval", "1.0"), 1.0)
    zero_index = _to_float(meta.get("Zero Index", "0"), 0.0)
    first_sample_time_s = -zero_index * xinc

    # Normalize metadata keys so plotting code can stay shared.
    norm_meta = {
        "channel": channel,
        "xinc": xinc,
        "first_sample_time_s": first_sample_time_s,
        "record_length": int(_to_float(meta.get("Record Length", "0"), 0.0)),
        "source_format": "tek_csv",
    }

    # Parse acquisition blocks: Labels,<ch> + TIME,<ch> + numeric rows.
    acquisitions = []
    while idx < len(lines):
        if lines[idx].strip() == "":
            idx += 1
            continue
        if not lines[idx].startswith("Labels,"):
            idx += 1
            continue

        # Skip "Labels,<ch>" and "TIME,<ch>" lines if present.
        idx += 1
        if idx < len(lines) and lines[idx].startswith("TIME,"):
            idx += 1

        yvals = []
        while idx < len(lines):
            raw = lines[idx].strip()
            if raw == "":
                break
            if raw.startswith("Labels,"):
                break
            parts = [p.strip() for p in lines[idx].split(",")]
            if len(parts) >= 2:
                yvals.append(_to_float(parts[1], np.nan))
            idx += 1

        if yvals:
            acquisitions.append(np.array(yvals, dtype=float))

    if not acquisitions:
        raise ValueError(f"{path}: no waveform acquisition blocks found")

    n_pts = min(arr.size for arr in acquisitions)
    if n_pts <= 0:
        raise ValueError(f"{path}: empty acquisition data")

    data = np.vstack([arr[:n_pts] for arr in acquisitions])
    return norm_meta, data


def load_waveform_file(path: str):
    with open(path, "r") as f:
        first = f.readline()

    if first.startswith("#"):
        meta, data = load_dat(path)
        meta = dict(meta)
        meta["source_format"] = "legacy_dat"
        return meta, data
    if first.startswith("Model,"):
        return load_tek_csv(path)

    raise ValueError(f"{path}: unknown format (expected legacy DAT or Tek CSV)")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Plot MSO58LP waveform file (.dat or Tek CSV)")
    ap.add_argument("file")
    ap.add_argument("--max", type=int, default=0, help="max acquisitions to draw")
    ap.add_argument("--mean", action="store_true", help="overlay mean trace")
    ap.add_argument("--time", action="store_true", help="use time axis (seconds)")
    ap.add_argument("--save", default=None, help="save figure to path")
    ap.add_argument("--no-show", action="store_true", help="do not display window")
    args = ap.parse_args(argv)

    import matplotlib
    if args.no_show and not args.save:
        args.save = os.path.splitext(args.file)[0] + ".png"
    if args.no_show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    meta, data = load_waveform_file(args.file)
    n_acq, n_pts = data.shape

    show = data if args.max <= 0 else data[: args.max]

    if args.time:
        xinc = float(meta.get("xinc", 1.0))
        t0 = float(meta.get("first_sample_time_s", 0.0))
        x = t0 + np.arange(n_pts) * xinc
        xlabel = "Time (s)"
    else:
        x = np.arange(n_pts)
        xlabel = "Sample index"

    fig, ax = plt.subplots(figsize=(11, 5))
    for i, row in enumerate(show):
        ax.plot(x, row, lw=0.6, alpha=0.5, label=f"acq {i}" if n_acq <= 12 else None)

    if args.mean:
        ax.plot(x, data.mean(axis=0), color="k", lw=1.4, label="mean")

    ch = meta.get("channel", "?")
    ts = meta.get("timestamp", "")
    ax.set_title(f"{os.path.basename(args.file)}  ch={ch}  "
                 f"{show.shape[0]}/{n_acq} acq × {n_pts} pts  {ts}")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Voltage (V)")
    ax.grid(True, alpha=0.3)
    if n_acq <= 12 or args.mean:
        ax.legend(loc="best", fontsize=8)
    fig.tight_layout()

    print("metadata:")
    for k, v in meta.items():
        print(f"  {k} = {v}")
    print(f"format: {meta.get('source_format', 'unknown')}")
    print(f"data shape: {data.shape}  (acquisitions × samples)")

    if args.save:
        fig.savefig(args.save, dpi=120)
        print(f"saved figure to {args.save}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    sys.exit(main())

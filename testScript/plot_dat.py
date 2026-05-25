#!/usr/bin/env python3
"""
plot_dat.py - Plot waveforms saved by the MSO58LP IOC store subsystem.

File format (produced by tekStoreWaveform.c):
    Line 1 : "# key=value key=value ..." metadata header
             keys: channel, xinc, ymult, yzero, yoff, npts, data_start,
                   first_sample_time_s, num_acq, timestamp
    Lines 2..N : one acquisition per line, comma-separated voltage samples.

Usage:
    python3 plot_dat.py FILE.dat [--max N] [--mean] [--save out.png]
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


def main(argv=None):
    ap = argparse.ArgumentParser(description="Plot MSO58LP .dat waveform file")
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

    meta, data = load_dat(args.file)
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
    print(f"data shape: {data.shape}  (acquisitions × samples)")

    if args.save:
        fig.savefig(args.save, dpi=120)
        print(f"saved figure to {args.save}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    sys.exit(main())

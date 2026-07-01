#!/usr/bin/env python3
"""
AD7606 Data Parser & Plotter

Reads per-channel binary files from an output directory and plots 4 channels (ch1~ch4).
Data format: each file = int16 LE samples for one channel.

Directory layout:
    <data_dir>/
        ch1.bin   -- int16 LE, channel 1 samples
        ch2.bin   -- int16 LE, channel 2 samples
        ch3.bin   -- int16 LE, channel 3 samples
        ch4.bin   -- int16 LE, channel 4 samples

Usage:
    python ad7606_plot.py [--dir ad7606_data] [--start 0] [--samples 4096]
                          [--layout subplot|overlay] [--save plot.png]
"""

import numpy as np
import argparse
from pathlib import Path

SAMPLE_RATE_HZ = 100_000
CHANNELS = 4


def load_per_channel_files(data_dir: str, start_sample: int = 0, num_samples: int = -1):
    """Load per-channel binary files into (samples x 4) int16 array."""
    root = Path(data_dir)
    channel_data = []

    total_samples = None
    for ch in range(CHANNELS):
        fpath = root / f"ch{ch + 1}.bin"
        if not fpath.exists():
            raise FileNotFoundError(f"Channel file not found: {fpath}")
        ch_arr = np.fromfile(fpath, dtype=np.int16)
        if total_samples is None:
            total_samples = ch_arr.shape[0]
        elif ch_arr.shape[0] != total_samples:
            print(
                f"[WARN] CH{ch + 1} length ({ch_arr.shape[0]}) differs from "
                f"expected ({total_samples}). Truncating to minimum."
            )
            total_samples = min(total_samples, ch_arr.shape[0])
        channel_data.append(ch_arr)

    # Truncate all channels to same length
    for ch in range(CHANNELS):
        channel_data[ch] = channel_data[ch][:total_samples]

    if num_samples <= 0 or start_sample + num_samples > total_samples:
        num_samples = total_samples - start_sample

    arr = np.zeros((num_samples, CHANNELS), dtype=np.int16)
    for ch in range(CHANNELS):
        arr[:, ch] = channel_data[ch][start_sample : start_sample + num_samples]

    return arr


def plot_subplots(arr: np.ndarray, sr: int):
    """4 separate subplots."""
    import matplotlib.pyplot as plt

    t = np.arange(arr.shape[0]) * 1000.0 / sr  # ms

    fig, axes = plt.subplots(4, 1, figsize=(14, 10), sharex=True)
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]

    for ch in range(4):
        ax = axes[ch]
        ax.plot(t, arr[:, ch], linewidth=0.5, color=colors[ch])
        ax.set_ylabel(f"CH{ch+1} (ADC code)")
        ax.grid(True, alpha=0.3)
        mean_val = np.mean(arr[:, ch])
        ax.axhline(mean_val, color="grey", linestyle="--", alpha=0.5,
                   label=f"mean={mean_val:.1f}")
        ax.legend(fontsize=8, loc="upper right")

    axes[-1].set_xlabel("Time (ms)")
    fig.suptitle(f"AD7606 4-CH Data  ({arr.shape[0]} samples @ {sr/1000:.0f} kSPS)",
                 fontsize=13, fontweight="bold")
    plt.tight_layout()
    return fig


def plot_overlay(arr: np.ndarray, sr: int):
    """All 4 channels overlaid."""
    import matplotlib.pyplot as plt

    t = np.arange(arr.shape[0]) * 1000.0 / sr  # ms
    fig, ax = plt.subplots(figsize=(14, 6))

    for ch in range(4):
        ax.plot(t, arr[:, ch], linewidth=0.5, label=f"CH{ch+1}", alpha=0.85)

    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("ADC code")
    ax.set_title(f"AD7606 4-CH Overlay  ({arr.shape[0]} samples @ {sr/1000:.0f} kSPS)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    return fig


def print_stats(arr: np.ndarray, sr: int):
    """Print per-channel statistics."""
    print(f"\n{'='*60}")
    print(f"  Samples: {arr.shape[0]}  |  Duration: {arr.shape[0]*1000/sr:.2f} ms")
    print(f"  Rate: {sr/1000:.0f} kSPS")
    print(f"{'='*60}")
    print(f"{'':>10} {'CH1':>10} {'CH2':>10} {'CH3':>10} {'CH4':>10}")
    print(f"{'-'*50}")
    print(f"{'Min':>10} {arr[:,0].min():>10d} {arr[:,1].min():>10d} "
          f"{arr[:,2].min():>10d} {arr[:,3].min():>10d}")
    print(f"{'Max':>10} {arr[:,0].max():>10d} {arr[:,1].max():>10d} "
          f"{arr[:,2].max():>10d} {arr[:,3].max():>10d}")
    print(f"{'Mean':>10} {arr[:,0].mean():>10.1f} {arr[:,1].mean():>10.1f} "
          f"{arr[:,2].mean():>10.1f} {arr[:,3].mean():>10.1f}")
    print(f"{'Std':>10} {arr[:,0].std():>10.1f} {arr[:,1].std():>10.1f} "
          f"{arr[:,2].std():>10.1f} {arr[:,3].std():>10.1f}")
    print(f"{'P-P':>10} {arr[:,0].max()-arr[:,0].min():>10d} {arr[:,1].max()-arr[:,1].min():>10d} "
          f"{arr[:,2].max()-arr[:,2].min():>10d} {arr[:,3].max()-arr[:,3].min():>10d}")
    print(f"{'='*60}\n")


def main():
    parser = argparse.ArgumentParser(description="AD7606 4-channel data plotter")
    parser.add_argument("--dir", default="ad7606_data",
                        help="Directory containing per-channel ch1.bin ~ ch4.bin files")
    parser.add_argument("--start", type=int, default=0, help="Start sample index")
    parser.add_argument("--samples", type=int, default=0,
                        help="Number of samples to plot (0=all)")
    parser.add_argument("--layout", choices=["subplot", "overlay"], default="subplot",
                        help="Plot layout style")
    parser.add_argument("--save", default="", help="Save plot to file (e.g. plot.png)")
    parser.add_argument("--stats-only", action="store_true", help="Only print stats, no plot")
    args = parser.parse_args()

    print(f"Loading data from directory: {args.dir}")
    arr = load_per_channel_files(args.dir, args.start, args.samples)
    print(f"Loaded {arr.shape[0]} samples x {arr.shape[1]} channels")

    print_stats(arr, SAMPLE_RATE_HZ)

    if not args.stats_only:
        if args.layout == "overlay":
            fig = plot_overlay(arr, SAMPLE_RATE_HZ)
        else:
            fig = plot_subplots(arr, SAMPLE_RATE_HZ)

        if args.save:
            fig.savefig(args.save, dpi=150, bbox_inches="tight")
            print(f"Plot saved to: {args.save}")
        else:
            import matplotlib.pyplot as plt
            plt.show()


if __name__ == "__main__":
    main()

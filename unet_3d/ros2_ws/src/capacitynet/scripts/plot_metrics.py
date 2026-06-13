#!/usr/bin/env python3
"""Post-hoc analysis of reachability map timing metrics.

Reads a CSV produced by metrics_recorder and generates a multi-panel figure.

Usage
-----
python3 plot_metrics.py ~/reachability_metrics.csv
python3 plot_metrics.py ~/reachability_metrics.csv --output report.png --target-fps 10
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use('Agg')  # headless-safe; switch to 'TkAgg' if you need a window
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

try:
    import pandas as pd
    _HAS_PANDAS = True
except ImportError:
    _HAS_PANDAS = False


# Colour palette consistent with the package
_STAGE_COLORS = {
    't_scatter_ms':       '#4e79a7',
    't_resize_ms':        '#59a14f',
    't_gpu_sync_ms':      '#f28e2b',
    't_prediction_ms':    '#e15759',
    't_cpu_transfer_ms':  '#76b7b2',
}
_STAGE_LABELS = {
    't_scatter_ms':       'Scatter',
    't_resize_ms':        'Resize',
    't_gpu_sync_ms':      'GPU sync',
    't_prediction_ms':    'Prediction',
    't_cpu_transfer_ms':  'CPU transfer',
}
_STAGES = list(_STAGE_COLORS.keys())


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def _load_csv(path: str) -> dict:
    """Return a dict of column_name → np.ndarray."""
    path = os.path.expanduser(path)
    if not os.path.isfile(path):
        sys.exit(f'File not found: {path}')

    if _HAS_PANDAS:
        df = pd.read_csv(path)
        return {col: df[col].to_numpy(dtype=float) for col in df.columns}

    # Pure-numpy fallback
    import csv
    with open(path) as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        sys.exit('CSV is empty.')
    data: dict[str, list] = {k: [] for k in rows[0]}
    for row in rows:
        for k, v in row.items():
            try:
                data[k].append(float(v))
            except ValueError:
                data[k].append(float('nan'))
    return {k: np.array(v) for k, v in data.items()}


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

def _reference_fps_line(ax, target_fps: float, color='red', alpha=0.7):
    budget_ms = 1000.0 / target_fps
    ax.axhline(budget_ms, color=color, linestyle='--', linewidth=1.2,
               alpha=alpha, label=f'{target_fps:.0f} fps budget ({budget_ms:.0f} ms)')


def _annotate_stats(ax, values: np.ndarray, unit='ms'):
    mu = np.nanmean(values)
    p95 = np.nanpercentile(values, 95)
    p99 = np.nanpercentile(values, 99)
    txt = f'mean={mu:.1f} {unit}  p95={p95:.1f}  p99={p99:.1f}'
    ax.text(0.01, 0.97, txt, transform=ax.transAxes,
            fontsize=7, va='top', ha='left',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))


# ---------------------------------------------------------------------------
# Panel builders
# ---------------------------------------------------------------------------

def _plot_latency_timeline(ax, data: dict, target_fps: float):
    frames = data.get('frame_id', np.arange(len(data['t_total_ms'])))
    total = data['t_total_ms']
    ax.plot(frames, total, linewidth=0.8, color='#333333', label='Total latency')
    _reference_fps_line(ax, target_fps)
    _annotate_stats(ax, total)
    ax.set_xlabel('Frame')
    ax.set_ylabel('Latency (ms)')
    ax.set_title('End-to-end latency over time')
    ax.legend(fontsize=7)
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(True, which='major', alpha=0.3)


def _plot_latency_histogram(ax, data: dict, target_fps: float):
    total = data['t_total_ms']
    ax.hist(total, bins=40, color='#4e79a7', edgecolor='white', linewidth=0.4)
    _reference_fps_line(ax, target_fps)
    _annotate_stats(ax, total)
    ax.set_xlabel('Latency (ms)')
    ax.set_ylabel('Count')
    ax.set_title('Latency distribution')
    ax.legend(fontsize=7)
    ax.grid(True, axis='y', alpha=0.3)


def _plot_stage_breakdown(ax, data: dict):
    means = {s: np.nanmean(data[s]) for s in _STAGES if s in data}
    labels = [_STAGE_LABELS[s] for s in means]
    values = list(means.values())
    colors = [_STAGE_COLORS[s] for s in means]

    bars = ax.barh(labels, values, color=colors, edgecolor='white', linewidth=0.5)
    for bar, val in zip(bars, values):
        ax.text(val + 0.1, bar.get_y() + bar.get_height() / 2,
                f'{val:.1f} ms', va='center', fontsize=7)

    total_mean = np.nanmean(data['t_total_ms'])
    ax.axvline(total_mean, color='black', linestyle=':', linewidth=1,
               label=f'Total mean: {total_mean:.1f} ms')
    ax.set_xlabel('Mean duration (ms)')
    ax.set_title('Per-stage mean breakdown')
    ax.legend(fontsize=7)
    ax.grid(True, axis='x', alpha=0.3)


def _plot_stage_boxplot(ax, data: dict):
    stage_data = [data[s] for s in _STAGES if s in data]
    labels = [_STAGE_LABELS[s] for s in _STAGES if s in data]
    colors = [_STAGE_COLORS[s] for s in _STAGES if s in data]

    bp = ax.boxplot(stage_data, vert=True, patch_artist=True,
                    medianprops=dict(color='black', linewidth=1.5),
                    flierprops=dict(marker='.', markersize=2, alpha=0.4))
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.8)

    ax.set_xticklabels(labels, rotation=15, ha='right', fontsize=8)
    ax.set_ylabel('Duration (ms)')
    ax.set_title('Per-stage distribution (box = IQR, whiskers = 1.5×IQR)')
    ax.grid(True, axis='y', alpha=0.3)


def _plot_gpu_utilization(ax, data: dict):
    if 'gpu_util_pct' not in data:
        ax.text(0.5, 0.5, 'No GPU utilization data', transform=ax.transAxes,
                ha='center', va='center')
        return
    frames = data.get('frame_id', np.arange(len(data['gpu_util_pct'])))
    util = data['gpu_util_pct']
    ax.plot(frames, util, linewidth=0.8, color='#e15759')
    ax.fill_between(frames, 0, util, alpha=0.15, color='#e15759')
    ax.set_ylim(0, 105)
    ax.set_xlabel('Frame')
    ax.set_ylabel('GPU utilization (%)')
    ax.set_title('GPU utilization')
    ax.grid(True, alpha=0.3)
    _annotate_stats(ax, util, unit='%')


def _plot_gpu_memory(ax, data: dict):
    if 'gpu_mem_used_mb' not in data:
        ax.text(0.5, 0.5, 'No GPU memory data', transform=ax.transAxes,
                ha='center', va='center')
        return
    frames = data.get('frame_id', np.arange(len(data['gpu_mem_used_mb'])))
    used = data['gpu_mem_used_mb']
    total = data.get('gpu_mem_total_mb', np.array([]))
    ax.plot(frames, used, linewidth=0.8, color='#59a14f', label='Used')
    ax.fill_between(frames, 0, used, alpha=0.15, color='#59a14f')
    if total.size > 0 and np.nanmax(total) > 0:
        mean_total = np.nanmean(total)
        ax.axhline(mean_total, color='#aaa', linestyle='--', linewidth=1,
                   label=f'Total ({mean_total:.0f} MB)')
    ax.set_xlabel('Frame')
    ax.set_ylabel('GPU memory (MB)')
    ax.set_title('GPU memory usage')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    _annotate_stats(ax, used, unit='MB')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def build_figure(data: dict, target_fps: float) -> plt.Figure:
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('Reachability Map — Real-time Performance Report', fontsize=13,
                 fontweight='bold', y=0.98)

    _plot_latency_timeline(axes[0, 0], data, target_fps)
    _plot_latency_histogram(axes[0, 1], data, target_fps)
    _plot_stage_breakdown(axes[1, 0], data)
    _plot_stage_boxplot(axes[1, 1], data)
    _plot_gpu_utilization(axes[2, 0], data)
    _plot_gpu_memory(axes[2, 1], data)

    fig.tight_layout(rect=[0, 0, 1, 0.97])
    return fig


def main():
    parser = argparse.ArgumentParser(
        description='Plot reachability map timing metrics from a CSV file.')
    parser.add_argument('csv', help='Path to the CSV produced by metrics_recorder')
    parser.add_argument('--output', '-o', default='',
                        help='Output image path (default: <csv_stem>_report.png)')
    parser.add_argument('--target-fps', type=float, default=10.0,
                        help='Reference FPS budget line (default: 10)')
    parser.add_argument('--show', action='store_true',
                        help='Display the figure interactively (requires a display)')
    parser.add_argument('--max-samples', type=int, default=100,
                        help='Use only the first N rows (default: 100). '
                             'Set to 0 to use all data.')
    args = parser.parse_args()

    data = _load_csv(args.csv)

    n_total = len(data.get('t_total_ms', []))
    if n_total == 0:
        sys.exit('No data rows found in CSV.')

    n_frames = n_total
    if args.max_samples > 0 and n_total > args.max_samples:
        data = {k: v[:args.max_samples] for k, v in data.items()}
        n_frames = args.max_samples
        print(f'Loaded {n_total} frames, using first {n_frames} (--max-samples {args.max_samples})')
    else:
        print(f'Loaded {n_frames} frames from {args.csv}')

    fig = build_figure(data, args.target_fps)

    output = args.output or os.path.splitext(
        os.path.abspath(os.path.expanduser(args.csv)))[0] + '_report.png'
    fig.savefig(output, dpi=150, bbox_inches='tight')
    print(f'Report saved → {output}')

    if args.show:
        matplotlib.use('TkAgg')
        plt.show()

    # Print a concise text summary
    total = data['t_total_ms']
    print(
        f'\n--- Summary ({n_frames} frames) ---\n'
        f'  Total latency  mean={np.nanmean(total):.1f} ms  '
        f'std={np.nanstd(total):.1f}  '
        f'p50={np.nanpercentile(total,50):.1f}  '
        f'p95={np.nanpercentile(total,95):.1f}  '
        f'p99={np.nanpercentile(total,99):.1f}\n'
        f'  Mean FPS       {np.nanmean(data["fps"]):.2f}\n'
        f'  FPS budget     {1000/args.target_fps:.1f} ms  '
        f'({(total <= 1000/args.target_fps).mean()*100:.1f}% of frames within budget)'
    )
    for s in _STAGES:
        if s in data:
            print(f'  {_STAGE_LABELS[s]:<15} mean={np.nanmean(data[s]):.1f} ms  '
                  f'std={np.nanstd(data[s]):.1f}')


if __name__ == '__main__':
    main()

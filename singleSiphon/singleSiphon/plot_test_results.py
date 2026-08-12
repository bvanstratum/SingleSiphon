#!/usr/bin/env python3
"""
Loads one or more results_*.pkl files produced by TestRunnerNode.py and
plots f_bar/p_bar/energy_spent_J against frequency_hz - one point per
accepted trial plus a per-frequency mean line, so the frequency-sweep shape
is visible at a glance. Multiple runs overlay on the same axes for
comparison (e.g. before/after a hardware or firmware change).

Not a ROS node (no rclpy) - the pickle is plain pandas output, so this runs
standalone, no colcon build/source needed.

Usage:
  # Single file, quick look:
  python3 plot_test_results.py /path/to/results_20260810_123456.pkl

  # Multiple named/colored runs: edit the RUNS list below instead (takes
  # priority over the CLI argument when non-empty).
"""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

# Render all plot text through a real LaTeX interpreter (requires latex,
# dvipng, ghostscript on PATH) instead of matplotlib's built-in mathtext -
# proper \bar{} etc. and consistent font with any LaTeX writeup this ends
# up feeding into.
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = 'serif'


def tex_escape(s):
    # Labels can come from RUNS entries or filenames typed/generated
    # elsewhere in this script - under usetex, a bare '_' starts LaTeX
    # subscript mode and throws if there's nothing valid after it, so any
    # string that isn't already a hand-written math/LaTeX literal (like the
    # PLOT_COLUMNS labels below) needs this before reaching matplotlib.
    return str(s).replace('_', r'\_')

# ── Runs to compare ─────────────────────────────────────────────────────
# One entry per results_*.pkl to overlay on the same axes. 'label' shows up
# in the legend, 'color' is any matplotlib color spec (name, hex, 'tab:blue',
# etc.) - leave as None to let matplotlib auto-assign. Empty list falls back
# to the pickle_path CLI argument (auto label/color, single run) instead.
RUNS = [
    # {'path': '/home/brian/SIPHION_Master_Folder/test_runner_results/results_20260810_150430.pkl',
    #  'label': 'baseline', 'color': 'tab:blue'},
    # {'path': '/home/brian/SIPHION_Master_Folder/test_runner_results/results_20260810_160000.pkl',
    #  'label': 'after tweak', 'color': 'tab:orange'},
    {'path': '/home/brian/SIPHION_Master_Folder/test_runner_results/results_20260810_155254.pkl', 'label': 'without grease', 'color': 'tab:blue'},
    {'path': '/home/brian/SIPHION_Master_Folder/test_runner_results/results_20260810_164726.pkl',
     'label': 'with grease', 'color': 'tab:green'},
    {'path': '/home/brian/SIPHION_Master_Folder/test_runner_results/results_20260811_152646.pkl', 'label': 'Bellows Attached', 'color': 'tab:purple'},
    {'path': '/home/brian/SIPHION_Master_Folder/test_runner_results/results_20260812_114105.pkl', 'label': 'In_Water', 'color': 'tab:red'},
]

PLOT_COLUMNS = [
    ('f_bar', r'$\bar{f}$ (mN)'),
    ('p_bar', r'$\bar{p}$ (W)'),
    ('energy_spent_J', 'Energy Spent (J)'),
]


def load_results(pickle_path):
    df = pd.read_pickle(pickle_path)
    # 'valid' is only present on data from TestRunnerNode.py's operator
    # accept/reject gate - bad runs are normally redone instead of ever
    # being written, so this is mostly a no-op safety net for old pickles
    # from before that feature, or a run someone accepted despite a flagged
    # warning.
    if 'valid' in df.columns:
        n_invalid = int((~df['valid'].fillna(True)).sum())
        if n_invalid:
            print(f'Dropping {n_invalid} row(s) flagged invalid (valid=False).')
        df = df[df['valid'].fillna(True)]
    return df


def plot_results(runs, title):
    """runs: list of (df, label, color) tuples, one per pickle - color may
    be None to let matplotlib auto-assign it (still consistent between a
    run's scatter and its mean line, since both draw from the same cycle)."""
    fig, axes = plt.subplots(len(PLOT_COLUMNS), 1, figsize=(8, 9), sharex=True)

    for df, label, color in runs:
        means = df.groupby('frequency_hz').mean(numeric_only=True)
        for ax, (col, ylabel) in zip(axes, PLOT_COLUMNS):
            if col not in df.columns:
                continue
            points = ax.scatter(df['frequency_hz'], df[col], alpha=0.4, color=color)
            # Reuse the scatter's actual color for the mean line, whether it
            # came from `color` above or matplotlib's own auto-assignment -
            # keeps a run's replicates/mean visually tied together even when
            # color is left as None.
            line_color = points.get_facecolor()[0]
            ax.plot(means.index, means[col], 'o-', color=line_color, label=tex_escape(label))
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)

    for ax in axes:
        if ax.has_data():
            ax.legend()
    axes[-1].set_xlabel('Frequency (Hz)')
    fig.suptitle(tex_escape(title))
    fig.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('pickle_path', nargs='?',
                         help='Path to a results_*.pkl file - ignored if RUNS (see top of script) is non-empty')
    args = parser.parse_args()

    run_specs = RUNS
    if not run_specs:
        if not args.pickle_path:
            print('No RUNS configured and no pickle_path given - see this script\'s '
                  'docstring/RUNS list.', file=sys.stderr)
            sys.exit(1)
        run_specs = [{'path': args.pickle_path, 'label': Path(args.pickle_path).name, 'color': None}]

    runs = []
    for spec in run_specs:
        pickle_path = Path(spec['path']).expanduser()
        if not pickle_path.exists():
            print(f'No such file: {pickle_path}', file=sys.stderr)
            sys.exit(1)
        df = load_results(pickle_path)
        if df.empty:
            print(f'No valid rows to plot in {pickle_path}.', file=sys.stderr)
            continue
        label = spec.get('label', pickle_path.name)
        print(f'\n=== {label} ({pickle_path.name}) ===')
        with pd.option_context('display.max_columns', None, 'display.width', None):
            print(df)
        runs.append((df, label, spec.get('color')))

    if not runs:
        print('No valid rows to plot in any run.', file=sys.stderr)
        sys.exit(1)

    title = ' vs '.join(label for _, label, _ in runs) if len(runs) > 1 else runs[0][1]
    plot_results(runs, title=title)
    plt.show()


if __name__ == '__main__':
    main()

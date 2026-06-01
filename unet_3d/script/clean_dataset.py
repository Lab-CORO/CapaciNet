"""
Scan formatted CapaciNet HDF5 files (datasets 'raw' + 'label') and flag
'non-interesting' samples — primarily reachability maps that are empty or
near-empty (failed/degenerate data generation).

SAFE BY DEFAULT: with no action flag it only LISTS what it would remove.
Use --action move (default quarantine) or --action delete to actually act.

Examples
--------
# Dry run: just report empty / near-empty files in val + train
python clean_dataset.py --dirs <exp>/data/val <exp>/data/train

# Quarantine (move) truly-empty files into a 'rejected/' sibling folder
python clean_dataset.py --dirs <exp>/data/train --action move

# Also treat files with <0.5% reachable voxels as non-interesting, then delete
python clean_dataset.py --dirs <exp>/data/train --min_reachable_frac 0.005 --action delete
"""
import argparse
import os
import shutil

import h5py
import numpy as np


def inspect(filepath, label_key, reachable_thresh):
    """Return stats for one file, or None if it is not a formatted raw/label file."""
    with h5py.File(filepath, "r") as f:
        if label_key not in f:
            return None
        label = f[label_key][:]
    label = label.astype(np.float32)
    reachable_frac = float(np.mean(label >= reachable_thresh))
    return {
        "max": float(label.max()),
        "mean": float(label.mean()),
        "reachable_frac": reachable_frac,
    }


def is_non_interesting(stats, min_reachable_frac, empty_eps):
    """A file is 'non-interesting' if its label is empty or below the reachable floor."""
    if stats["max"] < empty_eps:
        return True, "empty (max≈0)"
    if stats["reachable_frac"] < min_reachable_frac:
        return True, f"near-empty (reachable_frac={stats['reachable_frac']:.4f})"
    return False, ""


def main():
    ap = argparse.ArgumentParser(description="Flag/remove empty or near-empty CapaciNet HDF5 samples.")
    ap.add_argument("--dirs", nargs="+", required=True, help="Directories to scan for .h5 files")
    ap.add_argument("--label_key", default="label", help="HDF5 dataset name for the reachability map")
    ap.add_argument("--reachable_thresh", type=float, default=0.05,
                    help="A voxel counts as 'reachable' if label >= this (default 0.05)")
    ap.add_argument("--min_reachable_frac", type=float, default=0.0,
                    help="Flag files whose reachable-voxel fraction is below this "
                         "(default 0.0 = only flag totally empty files)")
    ap.add_argument("--empty_eps", type=float, default=1e-6, help="max(label) below this = empty")
    ap.add_argument("--action", choices=["list", "move", "delete"], default="list",
                    help="list (dry-run, default) | move to quarantine | delete")
    ap.add_argument("--quarantine_dir", default=None,
                    help="Where 'move' puts files (default: '<dir>/rejected/')")
    args = ap.parse_args()

    flagged, kept, total = [], 0, 0
    for d in args.dirs:
        if not os.path.isdir(d):
            print(f"WARNING: not a directory, skipping: {d}")
            continue
        for fn in sorted(os.listdir(d)):
            if not fn.endswith(".h5"):
                continue
            fp = os.path.join(d, fn)
            total += 1
            stats = inspect(fp, args.label_key, args.reachable_thresh)
            if stats is None:
                print(f"  SKIP (no '{args.label_key}' dataset): {fp}")
                continue
            bad, reason = is_non_interesting(stats, args.min_reachable_frac, args.empty_eps)
            if bad:
                flagged.append((fp, d, reason, stats))
            else:
                kept += 1

    print("\n" + "=" * 70)
    print(f"Scanned {total} file(s)  |  keep {kept}  |  flagged {len(flagged)}")
    print("=" * 70)
    for fp, d, reason, stats in flagged:
        print(f"  FLAG  {fp}")
        print(f"        {reason}  (max={stats['max']:.3f} mean={stats['mean']:.4f} "
              f"reachable_frac={stats['reachable_frac']:.4f})")

    if not flagged:
        print("Nothing to remove.")
        return

    if args.action == "list":
        print("\n(dry-run) Re-run with --action move  or  --action delete  to act on these.")
        return

    for fp, d, _, _ in flagged:
        if args.action == "delete":
            os.remove(fp)
            print(f"  DELETED {fp}")
        else:  # move
            qdir = args.quarantine_dir or os.path.join(d, "rejected")
            os.makedirs(qdir, exist_ok=True)
            dest = os.path.join(qdir, os.path.basename(fp))
            shutil.move(fp, dest)
            print(f"  MOVED   {fp} -> {dest}")

    print(f"\nDone: {args.action} applied to {len(flagged)} file(s).")


if __name__ == "__main__":
    main()

"""
CapaciNet – Evaluation script: regression metrics + 51-class (0–50) classification.

The reachability index is a fraction of 50 (n reachable orientations -> n/50), so
classification treats each voxel as one of 51 ordinal classes via
class = round(value * 50), applied to BOTH prediction and ground truth.

Usage (inside Apptainer container):
    python evaluate.py \
        --config     /workspace/unet_3d/experiments/<exp>/config.yaml \
        --checkpoint /workspace/unet_3d/experiments/<exp>/checkpoint/best_checkpoint.pytorch \
        --output_dir /workspace/unet_3d/experiments/<exp>/evaluation
"""
import argparse
import os

import h5py
import numpy as np
import torch
import yaml
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    _HAS_MATPLOTLIB = True
except ImportError:
    _HAS_MATPLOTLIB = False
    print("WARNING: matplotlib not found — plots will be skipped.")

from scipy.stats import pearsonr


# =============================================================================
#  MODEL LOADING
# =============================================================================

def load_model(config, checkpoint_path):
    from pytorch3dunet.unet3d.model import get_model

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    model = get_model(config["model"])
    state = torch.load(checkpoint_path, map_location=device)
    model.load_state_dict(state["model_state_dict"])
    model.to(device)
    model.eval()
    return model, device


# =============================================================================
#  INFERENCE
# =============================================================================

def normalize(arr):
    mean, std = arr.mean(), arr.std()
    if std < 1e-8:
        return arr - mean
    return (arr - mean) / std


def predict_file(model, device, filepath):
    """Return (pred, label, raw) as 3D float32 volumes (shape preserved for region masks)."""
    with h5py.File(filepath, "r") as f:
        raw   = f["raw"][:]
        label = f["label"][:]

    raw_f = raw.astype(np.float32)
    raw_norm = normalize(raw_f)
    tensor   = torch.from_numpy(raw_norm[np.newaxis, np.newaxis]).to(device)

    with torch.no_grad():
        pred = model(tensor).squeeze().cpu().numpy().astype(np.float32)

    return pred, label.astype(np.float32), raw_f


N_CLASSES = 51  # reachability index is a fraction of 50 -> classes 0..50


def to_classes(arr):
    """Quantize a [0,1] reachability fraction into integer classes 0..50.

    Each voxel's reachability is n/50 (n reachable orientations), so the natural
    class label is round(value * 50). Predictions are post-sigmoid (model in eval
    mode), but clip guards any tiny over/undershoot.
    """
    return np.clip(np.rint(arr * 50.0), 0, 50).astype(np.int16)


def gradmag3d(a):
    """Per-voxel gradient magnitude of a 3D array (for locating sharp/thin edges)."""
    gx, gy, gz = np.gradient(a.astype(np.float32))
    return np.sqrt(gx ** 2 + gy ** 2 + gz ** 2)


def region_masks(label_cls, label_cont, grad_pct=99.0):
    """Boolean masks (same shape as the volume) isolating where the model's job is
    actually hard. Aggregates dominated by empty background hide these.

      reachable : class >= 1            (any non-empty voxel)
      graded    : 1 <= class <= 49      (transition shell: not empty, not saturated)
      thin      : top-1% |grad(label)|  (sharp boundaries / singularity lines)
    """
    reachable = label_cls >= 1
    graded    = (label_cls >= 1) & (label_cls <= 49)
    gl = gradmag3d(label_cont)
    if reachable.any():
        thr = np.quantile(gl, grad_pct / 100.0)
        thin = gl >= thr
    else:
        thin = np.zeros_like(reachable)
    return {"reachable": reachable, "graded": graded, "thin": thin}


# =============================================================================
#  METRICS
# =============================================================================

def regression_metrics(preds, labels):
    diff = preds - labels
    mae  = float(np.mean(np.abs(diff)))
    rmse = float(np.sqrt(np.mean(diff ** 2)))
    r, _ = pearsonr(preds, labels)
    return {"MAE": mae, "RMSE": rmse, "Pearson_r": float(r)}


def confusion_matrix(pred_cls, label_cls, n_classes=N_CLASSES):
    """Dense (n_classes x n_classes) confusion matrix (rows=actual, cols=pred)."""
    idx = label_cls.astype(np.int64) * n_classes + pred_cls.astype(np.int64)
    cm = np.bincount(idx, minlength=n_classes * n_classes)
    return cm.reshape(n_classes, n_classes).astype(np.int64)


def quadratic_weighted_kappa(cm):
    """Cohen's quadratic-weighted kappa from a confusion matrix (ordinal agreement)."""
    n = cm.shape[0]
    total = cm.sum()
    if total == 0:
        return 0.0
    O = cm.astype(np.float64) / total                 # observed joint distribution
    act = O.sum(axis=1)                                # actual marginal
    prd = O.sum(axis=0)                                # predicted marginal
    E = np.outer(act, prd)                             # expected under independence
    i = np.arange(n)
    W = (i[:, None] - i[None, :]) ** 2 / float((n - 1) ** 2)
    denom = float((W * E).sum())
    if denom == 0:
        return 1.0
    return float(1.0 - (W * O).sum() / denom)


def multiclass_metrics(pred_cls, label_cls, n_classes=N_CLASSES):
    """Ordinal 51-class metrics measured on the integer-class arrays."""
    diff = np.abs(pred_cls.astype(np.int32) - label_cls.astype(np.int32))
    n = diff.size

    exact    = float(np.mean(diff == 0))
    within1  = float(np.mean(diff <= 1))
    within2  = float(np.mean(diff <= 2))
    within10 = float(np.mean(diff <= 10))
    within20 = float(np.mean(diff <= 20))
    mace    = float(np.mean(diff))          # mean absolute class error (orientation steps)
    medce   = float(np.median(diff))
    rmsece  = float(np.sqrt(np.mean(diff.astype(np.float64) ** 2)))

    cm = confusion_matrix(pred_cls, label_cls, n_classes)
    qwk = quadratic_weighted_kappa(cm)

    return {
        "ExactAccuracy": exact,
        "Within1Accuracy": within1,
        "Within2Accuracy": within2,
        "Within10Accuracy": within10,
        "Within20Accuracy": within20,
        "MeanClassError": mace,
        "MedianClassError": medce,
        "RMSEClass": rmsece,
        "QuadraticKappa": qwk,
        "N": int(n),
        "ConfusionMatrix": cm,
    }


def merge_class_metrics(cm, n_voxels):
    """Recompute scalar metrics from an accumulated confusion matrix.

    Lets the global aggregate be derived by summing per-file confusion matrices
    instead of concatenating every voxel.
    """
    n = cm.shape[0]
    i = np.arange(n)
    diff = np.abs(i[:, None] - i[None, :])             # (actual, pred) class distance
    counts = cm.astype(np.float64)
    total = counts.sum()
    if total == 0:
        total = 1.0

    exact    = float(np.trace(counts) / total)
    within1  = float(counts[diff <= 1].sum() / total)
    within2  = float(counts[diff <= 2].sum() / total)
    within10 = float(counts[diff <= 10].sum() / total)
    within20 = float(counts[diff <= 20].sum() / total)
    mace    = float((counts * diff).sum() / total)
    rmsece  = float(np.sqrt((counts * diff.astype(np.float64) ** 2).sum() / total))

    return {
        "ExactAccuracy": exact,
        "Within1Accuracy": within1,
        "Within2Accuracy": within2,
        "Within10Accuracy": within10,
        "Within20Accuracy": within20,
        "MeanClassError": mace,
        "MedianClassError": float("nan"),  # not recoverable from the matrix
        "RMSEClass": rmsece,
        "QuadraticKappa": quadratic_weighted_kappa(cm),
        "N": int(n_voxels),
        "ConfusionMatrix": cm,
    }


# =============================================================================
#  PLOTS
# =============================================================================

def plot_confusion_matrix(cls_metrics, output_path):
    if not _HAS_MATPLOTLIB:
        return
    from matplotlib.colors import LogNorm

    cm = cls_metrics["ConfusionMatrix"].astype(np.float64)
    n = cm.shape[0]

    fig, ax = plt.subplots(figsize=(7, 6))
    # Log colour: class 0 (empty space) dominates by orders of magnitude.
    im = ax.imshow(cm, origin="lower", cmap="viridis",
                   norm=LogNorm(vmin=1, vmax=max(cm.max(), 1)))
    ax.plot([0, n - 1], [0, n - 1], "r--", lw=0.8, alpha=0.6)  # perfect-agreement diagonal
    ax.set_xlabel("Predicted class (0–50)")
    ax.set_ylabel("Actual class (0–50)")
    ax.set_title(
        f"Confusion Matrix ({n} classes)  |  "
        f"Exact={cls_metrics['ExactAccuracy']:.3f}  "
        f"±1={cls_metrics['Within1Accuracy']:.3f}  "
        f"QWK={cls_metrics['QuadraticKappa']:.3f}",
        fontsize=10,
    )
    plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label="voxel count (log)")
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()


def error_cdf_from_cm(cm):
    """From a confusion matrix, return cdf[k] = fraction of voxels with
    |pred - label| <= k  (k = 0..n_classes-1). cdf[0] is the exact accuracy."""
    n = cm.shape[0]
    i = np.arange(n)
    dist = np.abs(i[:, None] - i[None, :]).ravel()
    counts = np.bincount(dist, weights=cm.ravel().astype(np.float64), minlength=n)
    total = counts.sum()
    return np.cumsum(counts) / (total if total > 0 else 1.0)


def plot_tolerance_curves(region_cm, region_n, output_path, thresh=0.95, kmax=15,
                          global_cm=None, global_n=None, ymin=0.1):
    """Cumulative accuracy vs tolerance: fraction of voxels within +-k classes,
    one curve per region (+ optional global). Marks where each curve first
    reaches `thresh` (95%)."""
    if not _HAS_MATPLOTLIB:
        return
    colors = {"global": "tab:green", "reachable": "tab:blue",
              "graded": "tab:orange", "thin": "tab:red"}
    ks = np.arange(0, kmax + 1)

    # global first (so it draws underneath the region curves), then the regions
    series = []
    if global_cm is not None:
        series.append(("global", global_cm, global_n))
    series += [(r, region_cm[r], region_n[r]) for r in region_cm]

    fig, ax = plt.subplots(figsize=(8, 5))
    for r, cm, n in series:
        cdf = error_cdf_from_cm(cm)
        k95 = int(np.argmax(cdf >= thresh)) if (cdf >= thresh).any() else None
        n0 = int(round(cdf[0] * n)) if n else 0   # exact-correct voxel count
        style = dict(marker="o", ms=3, color=colors.get(r))
        if r == "global":
            style.update(ls="--", lw=1.3, alpha=0.8)   # dashed to set it apart
        lbl = (f"{r}: exact={cdf[0]:.3f} ({n0:,} vox)"
               + (f", {thresh:.0%} at ±{k95}" if k95 is not None else f", <{thresh:.0%}"))
        ax.plot(ks, cdf[:kmax + 1], label=lbl, **style)
        if k95 is not None and k95 <= kmax:
            ax.axvline(k95, color=colors.get(r), ls=":", lw=0.9, alpha=0.6)

    ax.axhline(thresh, color="k", ls="--", lw=1, alpha=0.7)
    ax.set_xlabel("tolerance  ±k classes   (|pred − label| ≤ k)")
    ax.set_ylabel("fraction of voxels within tolerance")
    ax.set_title("Cumulative accuracy vs tolerance, by region")
    ax.set_xlim(0, kmax); ax.set_ylim(ymin, 1.01)
    ax.set_xticks(np.arange(0, kmax + 1, 1))
    ax.grid(alpha=0.3); ax.legend(loc="lower right", fontsize=9)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()


def plot_error_histogram(preds, labels, output_path):
    if not _HAS_MATPLOTLIB:
        return
    errors = np.abs(preds - labels)
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist(errors, bins=100, color="steelblue", edgecolor="none", density=True)
    ax.axvline(errors.mean(), color="red",    linestyle="--", label=f"Mean  {errors.mean():.4f}")
    ax.axvline(np.median(errors), color="orange", linestyle="--", label=f"Median {np.median(errors):.4f}")
    ax.set_xlabel("|pred − label|")
    ax.set_ylabel("Density")
    ax.set_title("Absolute Error Distribution (all val voxels)")
    ax.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()


def plot_region_slices(samples, output_dir):
    """For each (pred, label, raw) sample, save a regions_N.png showing:
      col 0: raw input (binary obstacle occupancy, 0/1)
      col 1: label where reachable (class>=1), black outside
      col 2: label where graded (class 1-49), black outside
      col 3: label where thin (top-1% |grad(label)|), black outside

    Uses the X-axis slice with the most thin-region voxels (singularity plane).
    """
    if not _HAS_MATPLOTLIB:
        return
    col_titles = ["raw input\n(obstacles)", "reachable\n(class ≥ 1)",
                  "graded\n(class 1–49)", "thin\n(top 1% |∇label|)"]
    for n, (pred, label, raw) in enumerate(samples):
        label_cls = to_classes(label)
        masks = region_masks(label_cls, label)

        # pick X slice with most thin-region voxels (singularity plane)
        thin_per_x = masks["thin"].sum(axis=(1, 2))
        xc = int(np.argmax(thin_per_x)) if thin_per_x.max() > 0 else label.shape[0] // 2

        raw_sl   = np.rot90(raw[xc, :, :])
        label_sl = np.rot90(label[xc, :, :])
        mask_sls = {r: np.rot90(masks[r][xc, :, :]) for r in ["reachable", "graded", "thin"]}

        fig, axs = plt.subplots(2, 2, figsize=(10, 10))
        axs_flat = axs.flatten()   # [top-left, top-right, bottom-left, bottom-right]

        # top-left: raw input — binary occupancy (0 or 1), hot colormap
        axs_flat[0].imshow(raw_sl, cmap="hot", vmin=0, vmax=1)
        axs_flat[0].set_title(col_titles[0]); axs_flat[0].axis("off")
        n_obs = int((raw_sl > 0).sum())
        axs_flat[0].set_xlabel(f"{n_obs} occupied voxels", fontsize=8)

        # top-right + bottom row: label value inside region, white outside
        cmap_region = plt.cm.gnuplot.copy(); cmap_region.set_bad("white")
        for ax, r, title in zip(axs_flat[1:], ["reachable", "graded", "thin"], col_titles[1:]):
            m = mask_sls[r]
            img = np.where(m, label_sl.astype(np.float32), np.nan)
            im = ax.imshow(img, cmap=cmap_region, vmin=0, vmax=1)
            ax.set_title(title); ax.axis("off")
            ax.set_xlabel(f"{int(m.sum())} voxels in slice", fontsize=8)
            plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

        plt.suptitle(f"Sample {n}  —  X slice {xc}  (singularity plane)", fontsize=11)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f"regions_{n}.png"), dpi=150)
        plt.close()


# =============================================================================
#  REPORT
# =============================================================================

def print_header(title):
    sep = "=" * 64
    print(f"\n{sep}\n  {title}\n{sep}")


def print_metrics(reg, cls):
    print(f"\n  -- Regression (continuous) --")
    print(f"  MAE       : {reg['MAE']:.6f}")
    print(f"  RMSE      : {reg['RMSE']:.6f}")
    print(f"  Pearson r : {reg['Pearson_r']:.6f}")
    print(f"\n  -- Classification (51 classes, 0–50) --")
    print(f"  Exact accuracy   : {cls['ExactAccuracy']:.4f}")
    print(f"  Within-±1 acc.   : {cls['Within1Accuracy']:.4f}")
    print(f"  Within-±2 acc.   : {cls['Within2Accuracy']:.4f}")
    print(f"  Within-±10 acc.  : {cls['Within10Accuracy']:.4f}")
    print(f"  Within-±20 acc.  : {cls['Within20Accuracy']:.4f}")
    print(f"  Mean class error : {cls['MeanClassError']:.4f}  (orientation steps)")
    print(f"  RMSE (class)     : {cls['RMSEClass']:.4f}")
    print(f"  Quadratic kappa  : {cls['QuadraticKappa']:.4f}")


_REGION_DESC = {
    "reachable": "class >= 1 (non-empty)",
    "graded":    "1..49 (transition shell)",
    "thin":      "top-1% |grad(label)| (edges/singularity lines)",
}


def print_region_metrics(region_cls, region_n):
    print(f"\n  -- Per-region classification (background excluded) --")
    print(f"  {'region':<10} {'voxels':>12}  {'exact':>6} {'±1':>6} {'±2':>6} {'±10':>6} {'±20':>6} "
          f"{'meanErr':>8} {'rmse':>6} {'QWK':>6}")
    for r, m in region_cls.items():
        print(f"  {r:<10} {region_n[r]:>12,}  "
              f"{m['ExactAccuracy']:>6.3f} {m['Within1Accuracy']:>6.3f} "
              f"{m['Within2Accuracy']:>6.3f} {m['Within10Accuracy']:>6.3f} "
              f"{m['Within20Accuracy']:>6.3f} {m['MeanClassError']:>8.3f} "
              f"{m['RMSEClass']:>6.2f} {m['QuadraticKappa']:>6.3f}")
    print(f"     ({', '.join(f'{r}={_REGION_DESC[r]}' for r in region_cls)})")


def save_report(reg, cls, output_path, n_files, n_voxels, region_cls=None, region_n=None):
    with open(output_path, "w") as f:
        f.write("=== CapaciNet Evaluation Report ===\n\n")
        f.write(f"Val files : {n_files}\n")
        f.write(f"Voxels    : {n_voxels:,}\n\n")
        f.write("-- Regression (continuous) --\n")
        f.write(f"MAE       : {reg['MAE']:.6f}\n")
        f.write(f"RMSE      : {reg['RMSE']:.6f}\n")
        f.write(f"Pearson r : {reg['Pearson_r']:.6f}\n\n")
        f.write("-- Classification (51 classes, 0-50; class = round(value*50)) --\n")
        f.write(f"Exact accuracy   : {cls['ExactAccuracy']:.4f}\n")
        f.write(f"Within-+-1 acc.  : {cls['Within1Accuracy']:.4f}\n")
        f.write(f"Within-+-2 acc.  : {cls['Within2Accuracy']:.4f}\n")
        f.write(f"Within-+-10 acc. : {cls['Within10Accuracy']:.4f}\n")
        f.write(f"Within-+-20 acc. : {cls['Within20Accuracy']:.4f}\n")
        f.write(f"Mean class error : {cls['MeanClassError']:.4f}  (orientation steps)\n")
        f.write(f"RMSE (class)     : {cls['RMSEClass']:.4f}\n")
        f.write(f"Quadratic kappa  : {cls['QuadraticKappa']:.4f}\n")
        if region_cls:
            f.write("\n-- Per-region classification (background excluded) --\n")
            f.write(f"{'region':<10} {'voxels':>12}  {'exact':>6} {'+-1':>6} {'+-2':>6} {'+-10':>6} {'+-20':>6} "
                    f"{'meanErr':>8} {'rmse':>6} {'QWK':>6}\n")
            for r, m in region_cls.items():
                f.write(f"{r:<10} {region_n[r]:>12,}  "
                        f"{m['ExactAccuracy']:>6.3f} {m['Within1Accuracy']:>6.3f} "
                        f"{m['Within2Accuracy']:>6.3f} {m['Within10Accuracy']:>6.3f} "
                        f"{m['Within20Accuracy']:>6.3f} {m['MeanClassError']:>8.3f} "
                        f"{m['RMSEClass']:>6.2f} {m['QuadraticKappa']:>6.3f}\n")
            for r in region_cls:
                f.write(f"  {r}: {_REGION_DESC[r]}\n")


# =============================================================================
#  MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="CapaciNet evaluation: regression + 51-class (0–50) classification")
    parser.add_argument("--config",     required=True, help="Path to experiment config.yaml")
    parser.add_argument("--checkpoint", required=True, help="Path to best_checkpoint.pytorch")
    parser.add_argument("--output_dir", required=True, help="Directory where results are saved")
    parser.add_argument("--val_dir",    default=None,
                        help="Override val directory from config (useful when config has container paths)")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    with open(args.config) as f:
        config = yaml.safe_load(f)

    val_dirs = [args.val_dir] if args.val_dir else config["loaders"]["val"]["file_paths"]
    val_files = []
    for d in val_dirs:
        val_files += sorted(
            os.path.join(d, fn) for fn in os.listdir(d) if fn.endswith(".h5")
        )

    print_header(f"CapaciNet Evaluation  |  {len(val_files)} val files")
    print(f"  Checkpoint : {args.checkpoint}")
    print(f"  Classes    : {N_CLASSES} (0–50, class = round(value*50))")

    model, device = load_model(config, args.checkpoint)

    all_preds, all_labels = [], []
    global_cm = np.zeros((N_CLASSES, N_CLASSES), dtype=np.int64)
    # per-region accumulated confusion matrices + voxel counts
    region_names = ["reachable", "graded", "thin"]
    region_cm = {r: np.zeros((N_CLASSES, N_CLASSES), dtype=np.int64) for r in region_names}
    region_n  = {r: 0 for r in region_names}
    region_samples = []   # first 3 (pred, label, raw) volumes for region_slices plot

    for i, fp in enumerate(val_files):
        fname = os.path.basename(fp)
        pred, label, raw = predict_file(model, device, fp)     # 3D volumes
        all_preds.append(pred.ravel())
        all_labels.append(label.ravel())
        if i < 3:
            region_samples.append((pred, label, raw))

        # First treatment: quantize pred + GT to integer classes 0..50.
        pred_cls  = to_classes(pred)
        label_cls = to_classes(label)

        reg = regression_metrics(pred.ravel(), label.ravel())
        cls = multiclass_metrics(pred_cls.ravel(), label_cls.ravel())
        global_cm += cls["ConfusionMatrix"]

        # region-restricted confusion matrices (the hard, non-background voxels)
        for r, m in region_masks(label_cls, label).items():
            if m.any():
                region_cm[r] += confusion_matrix(pred_cls[m], label_cls[m])
                region_n[r]  += int(m.sum())

        print(f"  [{i+1:4d}/{len(val_files)}] {fname:<45} "
              f"MAE={reg['MAE']:.4f}  Exact={cls['ExactAccuracy']:.4f}  "
              f"±1={cls['Within1Accuracy']:.4f}  QWK={cls['QuadraticKappa']:.4f}")

    # Global aggregate
    all_preds  = np.concatenate(all_preds)
    all_labels = np.concatenate(all_labels)

    global_reg = regression_metrics(all_preds, all_labels)
    global_cls = merge_class_metrics(global_cm, n_voxels=len(all_preds))
    region_cls = {r: merge_class_metrics(region_cm[r], n_voxels=region_n[r]) for r in region_names}

    print_header("Global Results")
    print(f"  Val files : {len(val_files)}")
    print(f"  Voxels    : {len(all_preds):,}")
    print_metrics(global_reg, global_cls)
    print_region_metrics(region_cls, region_n)

    # Save outputs
    plot_confusion_matrix(global_cls, os.path.join(args.output_dir, "confusion_matrix.png"))
    plot_error_histogram(all_preds, all_labels, os.path.join(args.output_dir, "error_histogram.png"))
    plot_tolerance_curves(region_cm, region_n, os.path.join(args.output_dir, "tolerance_curve.png"),
                          global_cm=global_cm, global_n=len(all_preds))
    plot_region_slices(region_samples, args.output_dir)
    np.savez_compressed(os.path.join(args.output_dir, "confusion_matrices.npz"),
                        global_cm=global_cm, **{f"{r}_cm": region_cm[r] for r in region_names})
    save_report(global_reg, global_cls, os.path.join(args.output_dir, "report.txt"),
                n_files=len(val_files), n_voxels=len(all_preds),
                region_cls=region_cls, region_n=region_n)

    print(f"\n  Saved to: {args.output_dir}/")
    if _HAS_MATPLOTLIB:
        print(f"    confusion_matrix.png")
        print(f"    error_histogram.png")
        print(f"    tolerance_curve.png")
        print(f"    regions_0.png .. regions_{min(2, len(val_files)-1)}.png")
    else:
        print(f"    (plots skipped — matplotlib not available)")
    print(f"    confusion_matrices.npz")
    print(f"    report.txt\n")


if __name__ == "__main__":
    main()

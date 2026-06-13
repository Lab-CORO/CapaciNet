"""
CapaciNet – Evaluation script: regression metrics + 51-class (0–50) classification.

The reachability index is a fraction of 50 (n reachable orientations -> n/50), so
classification treats each voxel as one of 51 ordinal classes via
class = round(value * 50), applied to BOTH prediction and ground truth.

Usage:
    # Normal inference run (PyTorch fp32):
    python3 scripts/evaluation.py \
        --config     config/test_reach.yaml \
        --data-dir   config/calibration_data/ \
        --output-dir /tmp/eval_cbe/ \
        --max-samples 10

    # PyTorch fp16:
    python3 scripts/evaluation.py ... --fp16

    # TensorRT (engine path set in config YAML via trt_engine_path):
    python3 scripts/evaluation.py --config config/test_reach.yaml ...

    # All files (no limit):
    python3 scripts/evaluation.py ... --max-samples 0

    # Regenerate plots/report from a saved .npz (skip inference):
    python3 scripts/evaluation.py \
        --from-npz   /tmp/eval_cbe/confusion_matrices.npz \
        --output-dir /tmp/eval_cbe_replot/

    # Change slice axis/index for region images:
    python3 scripts/evaluation.py ... --slice-axis z --slice-idx 64

    # Use a specific file for region slice images (instead of first evaluated file):
    python3 scripts/evaluation.py ... --region-sample config/calibration_data/sample_42.h5

    # Combine --from-npz with a new region sample (reruns inference on that file only):
    python3 scripts/evaluation.py \
        --from-npz      /tmp/eval_cbe/confusion_matrices.npz \
        --output-dir    /tmp/eval_cbe_replot/ \
        --region-sample config/calibration_data/sample_42.h5
"""
import argparse
import glob
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

def load_model(config):
    """Load model from a test_reach.yaml-style config (contains 'model' + 'model_path')."""
    from pytorch3dunet.unet3d.model import get_model
    from pytorch3dunet.unet3d import utils

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}")

    model = get_model(config["model"])
    utils.load_checkpoint(config["model_path"], model)
    model.to(device)
    model.eval()
    return model, device


# =============================================================================
#  MODEL LOADING — TRT
# =============================================================================

def load_trt_model(config, device):
    """Return a TRTModel if trt_engine_path is set, else None (PyTorch default).

    If trt_engine_path IS set, the engine MUST load — a missing file, CPU
    device, or load/warmup failure raises instead of silently demoting to
    PyTorch. (Silent demotion previously masked a wrong-path bug that made
    every 'TRT' run secretly PyTorch.)
    """
    trt_path = config.get('trt_engine_path', None)
    if not trt_path:
        return None  # engine not requested -> PyTorch is the legitimate backend

    if str(device) == 'cpu':
        raise RuntimeError(
            f"trt_engine_path is set ({trt_path}) but CUDA is not available — "
            f"cannot run TensorRT on CPU.")
    if not os.path.isfile(trt_path):
        raise FileNotFoundError(
            f"trt_engine_path is set but the file does not exist: {trt_path}\n"
            f"  Real engines live in subdirs, e.g. "
            f"config/cbe_synth/unet3d.trt or config/cbe_synth/unet3d_int8.trt")

    try:
        from capacitynet.trt_model import TRTModel
    except ImportError:
        # Fallback for running the script without colcon install
        import sys
        _pkg_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'capacitynet')
        sys.path.insert(0, _pkg_dir)
        from trt_model import TRTModel

    trt = TRTModel(trt_path)          # deserialize errors propagate (hard fail)
    trt.warmup(spatial=128)
    torch.cuda.synchronize()
    print(f"TRT engine loaded and warmed up: {trt_path}")
    return trt


# =============================================================================
#  INFERENCE
# =============================================================================

def normalize(arr):
    """Match pytorch3dunet Normalize (min-max to [-1, 1]) used at training time.

    z-score here silently breaks BatchNorm models: their tiny running_var buffers
    (calibrated to the [-1,1] training distribution) amplify any input-distribution
    shift into garbage. GroupNorm renormalizes per instance so it tolerated the
    mismatch, which is what masked this bug on the gcr baselines.
    """
    mn, mx = float(arr.min()), float(arr.max())
    norm01 = (arr - mn) / (mx - mn + 1e-10)
    return np.clip(2.0 * norm01 - 1.0, -1.0, 1.0).astype(np.float32)


def predict_file(infer_fn, device, filepath):
    """Return (pred, label, raw) as 3D float32 volumes (shape preserved for region masks)."""
    with h5py.File(filepath, "r") as f:
        raw   = f["raw"][:]
        label = f["label"][:]

    raw_f = raw.astype(np.float32)
    raw_norm = normalize(raw_f)
    tensor   = torch.from_numpy(raw_norm[np.newaxis, np.newaxis]).to(device)

    with torch.no_grad():
        pred = infer_fn(tensor).squeeze().cpu().numpy().astype(np.float32)

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
    # float64 throughout: at N=4B, float32 accumulation loses precision and saturates
    # pearsonr to 1.000000 (terms ~1e-10 fall below float32 eps once sum > 1e-3).
    p = preds.astype(np.float64)
    l = labels.astype(np.float64)
    diff = p - l
    mae  = float(np.mean(np.abs(diff)))
    rmse = float(np.sqrt(np.mean(diff ** 2)))
    r, _ = pearsonr(p, l)
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


def plot_error_histogram(preds, labels, output_path, hist_data=None):
    """hist_data=(counts, edges, mean, median) lets this run without raw arrays (--from-npz)."""
    if not _HAS_MATPLOTLIB:
        return
    if hist_data is not None:
        counts, edges, mean_val, median_val = hist_data
    else:
        errors = np.abs(preds - labels)
        counts, edges = np.histogram(errors, bins=100, density=True)
        mean_val, median_val = float(errors.mean()), float(np.median(errors))
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.bar(edges[:-1], counts, width=np.diff(edges), align='edge',
           color="steelblue", edgecolor="none")
    ax.axvline(mean_val,   color="red",    linestyle="--", label=f"Mean   {mean_val:.4f}")
    ax.axvline(median_val, color="orange", linestyle="--", label=f"Median {median_val:.4f}")
    ax.set_xlabel("|pred − label|")
    ax.set_ylabel("Density")
    ax.set_title("Absolute Error Distribution (all val voxels)")
    ax.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()


_SLICE_AXIS_MAP = {'x': 0, 'y': 1, 'z': 2}


def plot_region_slices(samples, output_dir, slice_axis='x', slice_idx=None):
    """For each (pred, label, raw) sample, save a regions_N.png showing:
      panel 0: raw input (binary obstacle occupancy, 0/1)
      panel 1: ground truth label (continuous 0–1, full volume)
      panel 2: label masked to reachable region (class >= 1)
      panel 3: label masked to thin region (top-1% |grad(label)|)

    slice_axis: 'x', 'y', or 'z' (default 'x').
    slice_idx:  fixed integer index; if None, auto-selects the slice with most
                thin-region voxels along the chosen axis.
    """
    if not _HAS_MATPLOTLIB:
        return
    dim = _SLICE_AXIS_MAP.get(slice_axis, 0)
    sum_axes = tuple(i for i in range(3) if i != dim)
    cmap_label = plt.cm.gnuplot.copy()
    cmap_label.set_bad("white")

    for n, (pred, label, raw) in enumerate(samples):
        label_cls = to_classes(label)
        masks = region_masks(label_cls, label)

        if slice_idx is not None:
            idx = slice_idx
        else:
            thin_per_dim = masks["thin"].sum(axis=sum_axes)
            idx = int(np.argmax(thin_per_dim)) if thin_per_dim.max() > 0 else label.shape[dim] // 2

        def _sl(arr):
            return np.take(arr, idx, axis=dim)

        raw_sl   = np.rot90(_sl(raw))
        label_sl = np.rot90(_sl(label))
        mask_sls = {r: np.rot90(_sl(masks[r])) for r in ["reachable", "thin"]}

        fig, axs = plt.subplots(2, 2, figsize=(10, 10))
        axs_flat = axs.flatten()

        # Panel 0 — raw input
        axs_flat[0].imshow(raw_sl, cmap="hot", vmin=0, vmax=1)
        axs_flat[0].set_title("raw input\n(obstacles)"); axs_flat[0].axis("off")
        axs_flat[0].set_xlabel(f"{int((raw_sl > 0).sum())} occupied voxels", fontsize=8)

        # Panel 1 — ground truth (full slice, no mask)
        im1 = axs_flat[1].imshow(label_sl.astype(np.float32), cmap=cmap_label, vmin=0, vmax=1)
        axs_flat[1].set_title("ground truth\n(continuous 0–1)"); axs_flat[1].axis("off")
        axs_flat[1].set_xlabel(f"{int((label_sl > 0).sum())} reachable voxels in slice", fontsize=8)
        plt.colorbar(im1, ax=axs_flat[1], fraction=0.046, pad=0.04)

        # Panels 2–3 — masked regions
        for ax, r, title in zip(axs_flat[2:], ["reachable", "thin"],
                                ["reachable\n(class ≥ 1)", "thin\n(top 1% |∇label|)"]):
            m = mask_sls[r]
            img = np.where(m, label_sl.astype(np.float32), np.nan)
            im = ax.imshow(img, cmap=cmap_label, vmin=0, vmax=1)
            ax.set_title(title); ax.axis("off")
            ax.set_xlabel(f"{int(m.sum())} voxels in slice", fontsize=8)
            plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

        mode = "auto" if slice_idx is None else "fixed"
        plt.suptitle(f"Sample {n}  —  {slice_axis.upper()} slice {idx}  ({mode})", fontsize=11)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f"regions_{n}.png"), dpi=150)
        plt.close()


# =============================================================================
#  REPORT
# =============================================================================

def print_header(title):
    sep = "=" * 64
    print(f"\n{sep}\n  {title}\n{sep}")


def print_metrics(reg, cls, reg_stats=None):
    print(f"\n  -- Regression (continuous) --")
    if reg_stats:
        print(f"  MAE       : {reg['MAE']:.6f}  (mean={reg_stats['MAE'][0]:.6f} ± std={reg_stats['MAE'][1]:.6f})")
        print(f"  RMSE      : {reg['RMSE']:.6f}  (mean={reg_stats['RMSE'][0]:.6f} ± std={reg_stats['RMSE'][1]:.6f})")
        print(f"  Pearson r : {reg['Pearson_r']:.6f}  (mean={reg_stats['Pearson_r'][0]:.6f} ± std={reg_stats['Pearson_r'][1]:.6f})")
    else:
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


def print_region_regression_metrics(region_reg, region_reg_stats=None):
    print(f"\n  -- Per-region regression (continuous) --")
    if region_reg_stats:
        print(f"  {'region':<10}  {'MAE (agg)':>10}  {'mean±std':>20}  {'RMSE (agg)':>10}  {'mean±std':>20}  {'Pearson (agg)':>13}  {'mean±std':>20}")
        for r, m in region_reg.items():
            s = region_reg_stats.get(r, {})
            mae_s  = f"{s['MAE'][0]:.6f}±{s['MAE'][1]:.6f}"      if s else "-"
            rmse_s = f"{s['RMSE'][0]:.6f}±{s['RMSE'][1]:.6f}"    if s else "-"
            pr_s   = f"{s['Pearson_r'][0]:.6f}±{s['Pearson_r'][1]:.6f}" if s else "-"
            print(f"  {r:<10}  {m['MAE']:>10.6f}  {mae_s:>20}  {m['RMSE']:>10.6f}  {rmse_s:>20}  {m['Pearson_r']:>13.6f}  {pr_s:>20}")
    else:
        print(f"  {'region':<10}  {'MAE':>10} {'RMSE':>10} {'Pearson r':>10}")
        for r, m in region_reg.items():
            print(f"  {r:<10}  {m['MAE']:>10.6f} {m['RMSE']:>10.6f} {m['Pearson_r']:>10.6f}")
    print(f"     ({', '.join(f'{r}={_REGION_DESC[r]}' for r in region_reg)})")


def save_report(reg, cls, output_path, n_files, n_voxels, region_cls=None, region_n=None,
                region_reg=None, global_reg_stats=None, region_reg_stats=None, backend=None):
    def _fmt_stat(stats, key):
        if stats and key in stats:
            return f"  mean={stats[key][0]:.6f}  std={stats[key][1]:.6f}"
        return ""

    with open(output_path, "w") as f:
        f.write("=== CapaciNet Evaluation Report ===\n\n")
        f.write(f"Backend   : {backend}\n")
        f.write(f"Val files : {n_files}\n")
        f.write(f"Voxels    : {n_voxels:,}\n\n")
        f.write("-- Regression (continuous) --\n")
        f.write(f"MAE       : {reg['MAE']:.6f}{_fmt_stat(global_reg_stats, 'MAE')}\n")
        f.write(f"RMSE      : {reg['RMSE']:.6f}{_fmt_stat(global_reg_stats, 'RMSE')}\n")
        f.write(f"Pearson r : {reg['Pearson_r']:.6f}{_fmt_stat(global_reg_stats, 'Pearson_r')}\n\n")
        f.write("-- Classification (51 classes, 0-50; class = round(value*50)) --\n")
        f.write(f"Exact accuracy   : {cls['ExactAccuracy']:.4f}\n")
        f.write(f"Within-+-1 acc.  : {cls['Within1Accuracy']:.4f}\n")
        f.write(f"Within-+-2 acc.  : {cls['Within2Accuracy']:.4f}\n")
        f.write(f"Within-+-10 acc. : {cls['Within10Accuracy']:.4f}\n")
        f.write(f"Within-+-20 acc. : {cls['Within20Accuracy']:.4f}\n")
        f.write(f"Mean class error : {cls['MeanClassError']:.4f}  (orientation steps)\n")
        f.write(f"RMSE (class)     : {cls['RMSEClass']:.4f}\n")
        f.write(f"Quadratic kappa  : {cls['QuadraticKappa']:.4f}\n")
        if region_reg:
            f.write("\n-- Per-region regression (continuous, background excluded) --\n")
            f.write(f"{'region':<10}  {'MAE (agg)':>10}  {'mean MAE':>10}  {'std MAE':>10}"
                    f"  {'RMSE (agg)':>10}  {'mean RMSE':>10}  {'std RMSE':>10}"
                    f"  {'Pearson (agg)':>12}  {'mean Pear':>10}  {'std Pear':>10}\n")
            for r, m in region_reg.items():
                s = (region_reg_stats or {}).get(r, {})
                f.write(f"{r:<10}  {m['MAE']:>10.6f}  "
                        f"{s.get('MAE',(float('nan'),))[0]:>10.6f}  {s.get('MAE',(0,0))[1]:>10.6f}  "
                        f"{m['RMSE']:>10.6f}  "
                        f"{s.get('RMSE',(float('nan'),))[0]:>10.6f}  {s.get('RMSE',(0,0))[1]:>10.6f}  "
                        f"{m['Pearson_r']:>12.6f}  "
                        f"{s.get('Pearson_r',(float('nan'),))[0]:>10.6f}  {s.get('Pearson_r',(0,0))[1]:>10.6f}\n")
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
#  NPZ SAVE / LOAD
# =============================================================================

def _save_extended_npz(path, global_cm, region_cm, region_n, n_files, n_voxels,
                       global_reg, global_reg_per_file,
                       region_reg, region_reg_per_file,
                       region_samples, all_preds, all_labels, backend):
    """Save everything needed to regenerate all outputs without re-running inference."""
    region_names = list(region_cm.keys())
    errors = np.abs(all_preds - all_labels).astype(np.float32)
    hist_counts, hist_edges = np.histogram(errors, bins=100, density=True)

    arrays = dict(
        global_cm=global_cm,
        n_files=np.array(n_files),
        n_voxels=np.array(n_voxels),
        hist_counts=hist_counts, hist_edges=hist_edges,
        hist_mean=np.array(float(errors.mean())),
        hist_median=np.array(float(np.median(errors))),
        global_mae=np.array(global_reg['MAE']),
        global_rmse=np.array(global_reg['RMSE']),
        global_pearson=np.array(global_reg['Pearson_r']),
        global_mae_pf=np.array([d['MAE']       for d in global_reg_per_file]),
        global_rmse_pf=np.array([d['RMSE']      for d in global_reg_per_file]),
        global_pearson_pf=np.array([d['Pearson_r'] for d in global_reg_per_file]),
        backend=np.array(backend),
    )
    for r in region_names:
        arrays[f"{r}_cm"] = region_cm[r]
        arrays[f"{r}_n"]  = np.array(region_n[r])
        if r in region_reg:
            arrays[f"{r}_mae"]    = np.array(region_reg[r]['MAE'])
            arrays[f"{r}_rmse"]   = np.array(region_reg[r]['RMSE'])
            arrays[f"{r}_pearson"]= np.array(region_reg[r]['Pearson_r'])
        if region_reg_per_file.get(r):
            arrays[f"{r}_mae_pf"]    = np.array([d['MAE']       for d in region_reg_per_file[r]])
            arrays[f"{r}_rmse_pf"]   = np.array([d['RMSE']      for d in region_reg_per_file[r]])
            arrays[f"{r}_pearson_pf"]= np.array([d['Pearson_r'] for d in region_reg_per_file[r]])
    for i, (pred, label, raw) in enumerate(region_samples):
        arrays[f"sample_{i}_pred"]  = pred.astype(np.float32)
        arrays[f"sample_{i}_label"] = label.astype(np.float32)
        arrays[f"sample_{i}_raw"]   = raw.astype(np.float32)
    np.savez_compressed(path, **arrays)


def _load_from_npz(path):
    """Reconstruct all objects needed to run plots/report from an extended .npz."""
    region_names = ["reachable", "graded", "thin"]
    d = np.load(path, allow_pickle=False)

    global_cm  = d['global_cm']
    n_files    = int(d['n_files'])
    n_voxels   = int(d['n_voxels'])
    backend    = str(d['backend'])
    hist_data  = (d['hist_counts'], d['hist_edges'],
                  float(d['hist_mean']), float(d['hist_median']))

    global_reg = {'MAE': float(d['global_mae']),
                  'RMSE': float(d['global_rmse']),
                  'Pearson_r': float(d['global_pearson'])}

    def _stats(mean_arr, std_arr):
        return (float(np.mean(mean_arr)), float(np.std(mean_arr)))

    global_reg_stats = {
        'MAE':       _stats(d['global_mae_pf'],     None),
        'RMSE':      _stats(d['global_rmse_pf'],    None),
        'Pearson_r': _stats(d['global_pearson_pf'], None),
    }
    # override with correct std
    for k, key in [('MAE','global_mae_pf'), ('RMSE','global_rmse_pf'), ('Pearson_r','global_pearson_pf')]:
        arr = d[key]
        global_reg_stats[k] = (float(np.mean(arr)), float(np.std(arr)))

    region_n   = {r: int(d[f"{r}_n"]) for r in region_names}
    global_cls = merge_class_metrics(global_cm, n_voxels=n_voxels)
    region_cls = {r: merge_class_metrics(d[f"{r}_cm"], n_voxels=region_n[r])
                  for r in region_names}

    region_reg = {}
    region_reg_stats = {}
    for r in region_names:
        if f"{r}_mae" in d:
            region_reg[r] = {'MAE': float(d[f"{r}_mae"]),
                             'RMSE': float(d[f"{r}_rmse"]),
                             'Pearson_r': float(d[f"{r}_pearson"])}
        if f"{r}_mae_pf" in d:
            region_reg_stats[r] = {
                k: (float(np.mean(d[f"{r}_{fk}_pf"])), float(np.std(d[f"{r}_{fk}_pf"])))
                for k, fk in [('MAE','mae'), ('RMSE','rmse'), ('Pearson_r','pearson')]
            }

    region_samples = []
    for i in range(3):
        if f"sample_{i}_pred" not in d:
            break
        region_samples.append((d[f"sample_{i}_pred"],
                                d[f"sample_{i}_label"],
                                d[f"sample_{i}_raw"]))

    region_cm = {r: d[f"{r}_cm"] for r in region_names}
    return (global_reg, global_cls, global_reg_stats,
            region_reg, region_cls, region_reg_stats,
            region_n, region_samples, hist_data,
            n_files, n_voxels, backend,
            global_cm, region_cm)


# =============================================================================
#  HELPERS
# =============================================================================

def _build_infer_fn(config, fp16):
    """Load model/TRT and return (infer_fn, device, backend_label)."""
    model, device = load_model(config)
    trt_model = load_trt_model(config, device)
    if trt_model is not None:
        label = f"TensorRT  ({config['trt_engine_path']})"
        return (lambda x: trt_model.infer(x)), device, label
    if fp16:
        model.half()
        label = f"PyTorch fp16  ({config['model_path']})"
        return (lambda x: model(x.half())), device, label
    label = f"PyTorch fp32  ({config['model_path']})"
    return (lambda x: model(x)), device, label


# =============================================================================
#  MAIN
# =============================================================================

DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'test_reach.yaml'
)


def main():
    parser = argparse.ArgumentParser(description="CapaciNet evaluation: regression + 51-class (0–50) classification")
    parser.add_argument("--config",      default=DEFAULT_CONFIG,
                        help="Path to test_reach.yaml (contains model config + model_path)")
    parser.add_argument("--data-dir",    default=None,
                        help="Directory of H5 files with 'raw' and 'label' datasets (required unless --from-npz)")
    parser.add_argument("--output-dir",  required=True,
                        help="Directory where results are saved")
    parser.add_argument("--max-samples", type=int, default=100,
                        help="Max number of H5 files to evaluate (default: 100, 0 = all)")
    parser.add_argument("--fp16", action="store_true",
                        help="Run PyTorch model in float16 (ignored when using TRT — engine dtype is fixed at build time)")
    parser.add_argument("--from-npz",   default=None, metavar="NPZ_PATH",
                        help="Skip inference; reload data from a previously saved confusion_matrices.npz")
    parser.add_argument("--slice-axis", choices=['x', 'y', 'z'], default='x',
                        help="Axis for region slice images (default: x)")
    parser.add_argument("--slice-idx",  type=int, default=None,
                        help="Fixed slice index along --slice-axis (default: auto-select richest thin-region slice)")
    parser.add_argument("--region-sample", default=None, metavar="H5_PATH",
                        help="H5 file to use for region slice images (default: first file evaluated)")
    args = parser.parse_args()
    if args.from_npz is None and args.data_dir is None:
        parser.error("--data-dir is required unless --from-npz is specified")

    os.makedirs(args.output_dir, exist_ok=True)

    # ------------------------------------------------------------------
    # Fast path: reload from previously saved .npz, skip inference
    # ------------------------------------------------------------------
    if args.from_npz:
        (global_reg, global_cls, global_reg_stats,
         region_reg, region_cls, region_reg_stats,
         region_n, region_samples, hist_data,
         n_files, n_voxels, backend_label,
         global_cm, region_cm) = _load_from_npz(args.from_npz)

        region_names = ["reachable", "graded", "thin"]
        print_header(f"CapaciNet Evaluation (from npz)  |  {n_files} files")
        print(f"  NPZ        : {args.from_npz}")
        print(f"  Backend    : {backend_label}")
        print(f"  Classes    : {N_CLASSES} (0–50, class = round(value*50))")
        print_header("Global Results")
        print(f"  Val files : {n_files}")
        print(f"  Voxels    : {n_voxels:,}")
        print_metrics(global_reg, global_cls, reg_stats=global_reg_stats)
        print_region_regression_metrics(region_reg, region_reg_stats=region_reg_stats)
        print_region_metrics(region_cls, region_n)

        if args.region_sample:
            with open(args.config) as f:
                cfg = yaml.safe_load(f)
            print(f"  Loading model for region sample inference...")
            _infer_fn, _device, _ = _build_infer_fn(cfg, args.fp16)
            s_pred, s_label, s_raw = predict_file(_infer_fn, _device, args.region_sample)
            region_samples = [(s_pred, s_label, s_raw)]
            print(f"  Region sample : {args.region_sample}")

        plot_confusion_matrix(global_cls, os.path.join(args.output_dir, "confusion_matrix.png"))
        plot_error_histogram(None, None, os.path.join(args.output_dir, "error_histogram.png"),
                             hist_data=hist_data)
        plot_tolerance_curves(region_cm, region_n,
                              os.path.join(args.output_dir, "tolerance_curve.png"),
                              global_cm=global_cm, global_n=n_voxels)
        plot_region_slices(region_samples, args.output_dir,
                           slice_axis=args.slice_axis, slice_idx=args.slice_idx)
        save_report(global_reg, global_cls, os.path.join(args.output_dir, "report.txt"),
                    n_files=n_files, n_voxels=n_voxels,
                    region_cls=region_cls, region_n=region_n,
                    region_reg=region_reg, global_reg_stats=global_reg_stats,
                    region_reg_stats=region_reg_stats, backend=backend_label)
        print(f"\n  Saved to: {args.output_dir}/\n")
        return

    # ------------------------------------------------------------------
    # Normal inference path
    # ------------------------------------------------------------------
    with open(args.config) as f:
        config = yaml.safe_load(f)

    val_files = sorted(glob.glob(os.path.join(args.data_dir, '**/*.h5'), recursive=True))
    if not val_files:
        val_files = sorted(glob.glob(os.path.join(args.data_dir, '*.h5')))
    if args.max_samples > 0:
        val_files = val_files[:args.max_samples]

    print_header(f"CapaciNet Evaluation  |  {len(val_files)} files")
    print(f"  Config     : {args.config}")
    print(f"  Checkpoint : {config['model_path']}")
    print(f"  Data dir   : {args.data_dir}")
    print(f"  Classes    : {N_CLASSES} (0–50, class = round(value*50))")

    infer_fn, device, backend_label = _build_infer_fn(config, args.fp16)
    print(f"  Backend    : {backend_label}")

    all_preds, all_labels = [], []
    global_cm = np.zeros((N_CLASSES, N_CLASSES), dtype=np.int64)
    # per-region accumulated confusion matrices + voxel counts
    region_names = ["reachable", "graded", "thin"]
    region_cm     = {r: np.zeros((N_CLASSES, N_CLASSES), dtype=np.int64) for r in region_names}
    region_n      = {r: 0 for r in region_names}
    region_preds  = {r: [] for r in region_names}   # continuous pred voxels per region
    region_labels = {r: [] for r in region_names}   # continuous label voxels per region
    region_samples = []   # first 3 (pred, label, raw) volumes for region_slices plot
    global_reg_per_file  = []                        # per-file global regression dicts
    region_reg_per_file  = {r: [] for r in region_names}  # per-file per-region regression dicts

    for i, fp in enumerate(val_files):
        fname = os.path.basename(fp)
        pred, label, raw = predict_file(infer_fn, device, fp)   # 3D volumes
        all_preds.append(pred.ravel())
        all_labels.append(label.ravel())
        if i < 3 and not args.region_sample:
            region_samples.append((pred, label, raw))

        # First treatment: quantize pred + GT to integer classes 0..50.
        pred_cls  = to_classes(pred)
        label_cls = to_classes(label)

        reg = regression_metrics(pred.ravel(), label.ravel())
        global_reg_per_file.append(reg)
        cls = multiclass_metrics(pred_cls.ravel(), label_cls.ravel())
        global_cm += cls["ConfusionMatrix"]

        # region-restricted confusion matrices + continuous pred/label for regression
        for r, m in region_masks(label_cls, label).items():
            if m.any():
                region_cm[r]    += confusion_matrix(pred_cls[m], label_cls[m])
                region_n[r]     += int(m.sum())
                region_preds[r].append(pred[m])
                region_labels[r].append(label[m])
                region_reg_per_file[r].append(regression_metrics(pred[m], label[m]))

        print(f"  [{i+1:4d}/{len(val_files)}] {fname:<45} "
              f"MAE={reg['MAE']:.4f}  Exact={cls['ExactAccuracy']:.4f}  "
              f"±1={cls['Within1Accuracy']:.4f}  QWK={cls['QuadraticKappa']:.4f}")

    if args.region_sample:
        s_pred, s_label, s_raw = predict_file(infer_fn, device, args.region_sample)
        region_samples = [(s_pred, s_label, s_raw)]
        print(f"  Region sample : {args.region_sample}")

    # Global aggregate
    all_preds  = np.concatenate(all_preds)
    all_labels = np.concatenate(all_labels)

    global_reg = regression_metrics(all_preds, all_labels)
    global_cls = merge_class_metrics(global_cm, n_voxels=len(all_preds))
    region_cls = {r: merge_class_metrics(region_cm[r], n_voxels=region_n[r]) for r in region_names}
    region_reg = {r: regression_metrics(np.concatenate(region_preds[r]),
                                        np.concatenate(region_labels[r]))
                  for r in region_names if region_preds[r]}

    def _reg_stats(per_file):
        """mean ± std across files for each regression key."""
        keys = ['MAE', 'RMSE', 'Pearson_r']
        return {k: (float(np.mean([d[k] for d in per_file])),
                    float(np.std( [d[k] for d in per_file])))
                for k in keys}

    global_reg_stats = _reg_stats(global_reg_per_file)
    region_reg_stats = {r: _reg_stats(region_reg_per_file[r])
                        for r in region_names if region_reg_per_file[r]}

    print_header("Global Results")
    print(f"  Val files : {len(val_files)}")
    print(f"  Voxels    : {len(all_preds):,}")
    print_metrics(global_reg, global_cls, reg_stats=global_reg_stats)
    print_region_regression_metrics(region_reg, region_reg_stats=region_reg_stats)
    print_region_metrics(region_cls, region_n)

    # Save outputs
    plot_confusion_matrix(global_cls, os.path.join(args.output_dir, "confusion_matrix.png"))
    plot_error_histogram(all_preds, all_labels, os.path.join(args.output_dir, "error_histogram.png"))
    plot_tolerance_curves(region_cm, region_n, os.path.join(args.output_dir, "tolerance_curve.png"),
                          global_cm=global_cm, global_n=len(all_preds))
    plot_region_slices(region_samples, args.output_dir,
                       slice_axis=args.slice_axis, slice_idx=args.slice_idx)
    _save_extended_npz(
        os.path.join(args.output_dir, "confusion_matrices.npz"),
        global_cm=global_cm, region_cm=region_cm, region_n=region_n,
        n_files=len(val_files), n_voxels=len(all_preds),
        global_reg=global_reg, global_reg_per_file=global_reg_per_file,
        region_reg=region_reg, region_reg_per_file=region_reg_per_file,
        region_samples=region_samples,
        all_preds=all_preds, all_labels=all_labels,
        backend=backend_label,
    )
    save_report(global_reg, global_cls, os.path.join(args.output_dir, "report.txt"),
                n_files=len(val_files), n_voxels=len(all_preds),
                region_cls=region_cls, region_n=region_n,
                region_reg=region_reg, global_reg_stats=global_reg_stats,
                region_reg_stats=region_reg_stats, backend=backend_label)

    print(f"\n  Saved to: {args.output_dir}/")
    if _HAS_MATPLOTLIB:
        print(f"    confusion_matrix.png")
        print(f"    error_histogram.png")
        print(f"    tolerance_curve.png")
        print(f"    regions_0.png .. regions_{min(2, len(val_files)-1)}.png")
    else:
        print(f"    (plots skipped — matplotlib not available)")
    print(f"    confusion_matrices.npz")
    print(f"    report.txt")
    print(f"  ({len(val_files)} files evaluated)\n")


if __name__ == "__main__":
    main()

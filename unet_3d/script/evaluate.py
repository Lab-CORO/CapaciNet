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
    with h5py.File(filepath, "r") as f:
        raw   = f["raw"][:]
        label = f["label"][:]

    raw_norm = normalize(raw.astype(np.float32))
    tensor   = torch.from_numpy(raw_norm[np.newaxis, np.newaxis]).to(device)

    with torch.no_grad():
        pred = model(tensor).squeeze().cpu().numpy()

    return pred.ravel(), label.ravel().astype(np.float32)


N_CLASSES = 51  # reachability index is a fraction of 50 -> classes 0..50


def to_classes(arr):
    """Quantize a [0,1] reachability fraction into integer classes 0..50.

    Each voxel's reachability is n/50 (n reachable orientations), so the natural
    class label is round(value * 50). Predictions are post-sigmoid (model in eval
    mode), but clip guards any tiny over/undershoot.
    """
    return np.clip(np.rint(arr * 50.0), 0, 50).astype(np.int16)


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

    exact   = float(np.mean(diff == 0))
    within1 = float(np.mean(diff <= 1))
    within2 = float(np.mean(diff <= 2))
    mace    = float(np.mean(diff))          # mean absolute class error (orientation steps)
    medce   = float(np.median(diff))
    rmsece  = float(np.sqrt(np.mean(diff.astype(np.float64) ** 2)))

    cm = confusion_matrix(pred_cls, label_cls, n_classes)
    qwk = quadratic_weighted_kappa(cm)

    return {
        "ExactAccuracy": exact,
        "Within1Accuracy": within1,
        "Within2Accuracy": within2,
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

    exact   = float(np.trace(counts) / total)
    within1 = float(counts[diff <= 1].sum() / total)
    within2 = float(counts[diff <= 2].sum() / total)
    mace    = float((counts * diff).sum() / total)
    rmsece  = float(np.sqrt((counts * diff.astype(np.float64) ** 2).sum() / total))

    return {
        "ExactAccuracy": exact,
        "Within1Accuracy": within1,
        "Within2Accuracy": within2,
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
    print(f"  Mean class error : {cls['MeanClassError']:.4f}  (orientation steps)")
    print(f"  RMSE (class)     : {cls['RMSEClass']:.4f}")
    print(f"  Quadratic kappa  : {cls['QuadraticKappa']:.4f}")


def save_report(reg, cls, output_path, n_files, n_voxels):
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
        f.write(f"Mean class error : {cls['MeanClassError']:.4f}  (orientation steps)\n")
        f.write(f"RMSE (class)     : {cls['RMSEClass']:.4f}\n")
        f.write(f"Quadratic kappa  : {cls['QuadraticKappa']:.4f}\n")


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
    per_file_rows = []

    for i, fp in enumerate(val_files):
        fname = os.path.basename(fp)
        preds, labels = predict_file(model, device, fp)
        all_preds.append(preds)
        all_labels.append(labels)

        # First treatment: quantize pred + GT to integer classes 0..50.
        pred_cls  = to_classes(preds)
        label_cls = to_classes(labels)

        reg = regression_metrics(preds, labels)
        cls = multiclass_metrics(pred_cls, label_cls)
        global_cm += cls["ConfusionMatrix"]
        per_file_rows.append((fname, reg, cls))

        print(f"  [{i+1:4d}/{len(val_files)}] {fname:<45} "
              f"MAE={reg['MAE']:.4f}  Exact={cls['ExactAccuracy']:.4f}  "
              f"±1={cls['Within1Accuracy']:.4f}  QWK={cls['QuadraticKappa']:.4f}")

    # Global aggregate
    all_preds  = np.concatenate(all_preds)
    all_labels = np.concatenate(all_labels)

    global_reg = regression_metrics(all_preds, all_labels)
    global_cls = merge_class_metrics(global_cm, n_voxels=len(all_preds))

    print_header("Global Results")
    print(f"  Val files : {len(val_files)}")
    print(f"  Voxels    : {len(all_preds):,}")
    print_metrics(global_reg, global_cls)

    # Save outputs
    plot_confusion_matrix(global_cls, os.path.join(args.output_dir, "confusion_matrix.png"))
    plot_error_histogram(all_preds, all_labels, os.path.join(args.output_dir, "error_histogram.png"))
    save_report(global_reg, global_cls, os.path.join(args.output_dir, "report.txt"),
                n_files=len(val_files), n_voxels=len(all_preds))

    print(f"\n  Saved to: {args.output_dir}/")
    if _HAS_MATPLOTLIB:
        print(f"    confusion_matrix.png")
        print(f"    error_histogram.png")
    else:
        print(f"    (plots skipped — matplotlib not available)")
    print(f"    report.txt\n")


if __name__ == "__main__":
    main()

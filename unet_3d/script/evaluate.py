"""
CapaciNet – Evaluation script: regression metrics + confusion matrix + IoU.

Usage (inside Apptainer container):
    python evaluate.py \
        --config     /workspace/unet_3d/experiments/<exp>/config.yaml \
        --checkpoint /workspace/unet_3d/experiments/<exp>/checkpoint/best_checkpoint.pytorch \
        --output_dir /workspace/unet_3d/experiments/<exp>/evaluation \
        --threshold  0.5
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


# =============================================================================
#  METRICS
# =============================================================================

def regression_metrics(preds, labels):
    diff = preds - labels
    mae  = float(np.mean(np.abs(diff)))
    rmse = float(np.sqrt(np.mean(diff ** 2)))
    r, _ = pearsonr(preds, labels)
    return {"MAE": mae, "RMSE": rmse, "Pearson_r": float(r)}


def classification_metrics(preds, labels, threshold=0.5):
    p_bin = (preds  >= threshold).astype(np.int8)
    l_bin = (labels >= threshold).astype(np.int8)

    tp = int(np.sum((p_bin == 1) & (l_bin == 1)))
    tn = int(np.sum((p_bin == 0) & (l_bin == 0)))
    fp = int(np.sum((p_bin == 1) & (l_bin == 0)))
    fn = int(np.sum((p_bin == 0) & (l_bin == 1)))

    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall    = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1        = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0
    iou       = tp / (tp + fp + fn) if (tp + fp + fn) > 0 else 0.0
    accuracy  = (tp + tn) / (tp + tn + fp + fn) if (tp + tn + fp + fn) > 0 else 0.0

    return {
        "TP": tp, "TN": tn, "FP": fp, "FN": fn,
        "Precision": precision, "Recall": recall,
        "F1": f1, "IoU": iou, "Accuracy": accuracy,
    }


# =============================================================================
#  PLOTS
# =============================================================================

def plot_confusion_matrix(cm_metrics, output_path):
    if not _HAS_MATPLOTLIB:
        return
    tp, tn = cm_metrics["TP"], cm_metrics["TN"]
    fp, fn = cm_metrics["FP"], cm_metrics["FN"]
    total  = tp + tn + fp + fn

    matrix = np.array([[tn, fp], [fn, tp]])
    labels = np.array([
        [f"TN\n{tn:,}\n({100*tn/total:.1f}%)", f"FP\n{fp:,}\n({100*fp/total:.1f}%)"],
        [f"FN\n{fn:,}\n({100*fn/total:.1f}%)", f"TP\n{tp:,}\n({100*tp/total:.1f}%)"],
    ])

    fig, ax = plt.subplots(figsize=(6, 5))
    im = ax.imshow(matrix, cmap="Blues")
    ax.set_xticks([0, 1]); ax.set_xticklabels(["Predicted\nNot Reachable", "Predicted\nReachable"])
    ax.set_yticks([0, 1]); ax.set_yticklabels(["Actual\nNot Reachable", "Actual\nReachable"])
    for i in range(2):
        for j in range(2):
            ax.text(j, i, labels[i, j], ha="center", va="center", fontsize=11,
                    color="white" if matrix[i, j] > matrix.max() * 0.6 else "black")
    ax.set_title(
        f"Confusion Matrix  |  IoU={cm_metrics['IoU']:.3f}  F1={cm_metrics['F1']:.3f}  "
        f"Acc={cm_metrics['Accuracy']:.3f}",
        fontsize=10,
    )
    plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
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
    print(f"\n  -- Regression --")
    print(f"  MAE       : {reg['MAE']:.6f}")
    print(f"  RMSE      : {reg['RMSE']:.6f}")
    print(f"  Pearson r : {reg['Pearson_r']:.6f}")
    print(f"\n  -- Classification (threshold 0.5) --")
    print(f"  TP: {cls['TP']:>12,}   FP: {cls['FP']:>12,}")
    print(f"  FN: {cls['FN']:>12,}   TN: {cls['TN']:>12,}")
    print(f"  Precision : {cls['Precision']:.4f}")
    print(f"  Recall    : {cls['Recall']:.4f}")
    print(f"  F1        : {cls['F1']:.4f}")
    print(f"  IoU       : {cls['IoU']:.4f}")
    print(f"  Accuracy  : {cls['Accuracy']:.4f}")


def save_report(reg, cls, output_path, n_files, n_voxels):
    with open(output_path, "w") as f:
        f.write("=== CapaciNet Evaluation Report ===\n\n")
        f.write(f"Val files : {n_files}\n")
        f.write(f"Voxels    : {n_voxels:,}\n\n")
        f.write("-- Regression --\n")
        f.write(f"MAE       : {reg['MAE']:.6f}\n")
        f.write(f"RMSE      : {reg['RMSE']:.6f}\n")
        f.write(f"Pearson r : {reg['Pearson_r']:.6f}\n\n")
        f.write("-- Classification (threshold 0.5) --\n")
        f.write(f"TP: {cls['TP']:,}   FP: {cls['FP']:,}\n")
        f.write(f"FN: {cls['FN']:,}   TN: {cls['TN']:,}\n")
        f.write(f"Precision : {cls['Precision']:.4f}\n")
        f.write(f"Recall    : {cls['Recall']:.4f}\n")
        f.write(f"F1        : {cls['F1']:.4f}\n")
        f.write(f"IoU       : {cls['IoU']:.4f}\n")
        f.write(f"Accuracy  : {cls['Accuracy']:.4f}\n")


# =============================================================================
#  MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description="CapaciNet evaluation: regression + confusion matrix + IoU")
    parser.add_argument("--config",     required=True, help="Path to experiment config.yaml")
    parser.add_argument("--checkpoint", required=True, help="Path to best_checkpoint.pytorch")
    parser.add_argument("--output_dir", required=True, help="Directory where results are saved")
    parser.add_argument("--threshold",  type=float, default=0.5, help="Binarization threshold (default: 0.5)")
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
    print(f"  Threshold  : {args.threshold}")

    model, device = load_model(config, args.checkpoint)

    all_preds, all_labels = [], []
    per_file_rows = []

    for i, fp in enumerate(val_files):
        fname = os.path.basename(fp)
        preds, labels = predict_file(model, device, fp)
        all_preds.append(preds)
        all_labels.append(labels)

        reg = regression_metrics(preds, labels)
        cls = classification_metrics(preds, labels, args.threshold)
        per_file_rows.append((fname, reg, cls))

        print(f"  [{i+1:4d}/{len(val_files)}] {fname:<45} "
              f"MAE={reg['MAE']:.4f}  IoU={cls['IoU']:.4f}  F1={cls['F1']:.4f}")

    # Global aggregate
    all_preds  = np.concatenate(all_preds)
    all_labels = np.concatenate(all_labels)

    global_reg = regression_metrics(all_preds, all_labels)
    global_cls = classification_metrics(all_preds, all_labels, args.threshold)

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

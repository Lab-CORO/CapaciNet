"""
Script de vérification de l'augmentation par rotation en Z.
Charge un sample HDF5, applique des rotations à différents angles,
et sauvegarde une figure comparative.
"""
import h5py
import numpy as np
from scipy.ndimage import rotate
import matplotlib.pyplot as plt

# --- Paramètres ---
H5_PATH = "/lustre06/project/6089348/willore/CapaciNet/unet_3d/data/train/25_10_2025_11_17_3_group_0.h5"
ANGLES = [-30, -15, 15, 30]  # angles à tester
OUTPUT_PATH = "/lustre06/project/6089348/willore/CapaciNet/unet_3d/verif_rotation_z.png"

# --- Chargement ---
with h5py.File(H5_PATH, "r") as f:
    raw = f["raw"][:]
    label = f["label"][:]

print(f"Raw shape: {raw.shape}, dtype: {raw.dtype}")
print(f"Raw min: {raw.min():.6f}, max: {raw.max():.6f}, mean: {raw.mean():.6f}")
print(f"Label shape: {label.shape}, dtype: {label.dtype}")
print(f"Label unique values (sample): {np.unique(label)[:10]}")

mid_z = raw.shape[0] // 2  # coupe au milieu selon Z (axe 0)

# Plage de valeurs pour un affichage cohérent entre les colonnes
raw_vmin, raw_vmax = np.percentile(raw, [1, 99])
label_vmin, label_vmax = label.min(), label.max()

print(f"\nRaw display range (percentile 1-99): [{raw_vmin:.4f}, {raw_vmax:.4f}]")
print(f"Label display range: [{label_vmin:.4f}, {label_vmax:.4f}]")

# --- Figure ---
n_angles = len(ANGLES)
fig, axes = plt.subplots(2, n_angles + 1, figsize=(4 * (n_angles + 1), 8))

# Colonne 0 : original
axes[0, 0].imshow(raw[mid_z], cmap="gray", vmin=raw_vmin, vmax=raw_vmax)
axes[0, 0].set_title("Raw original")
axes[1, 0].imshow(label[mid_z], cmap="gray", vmin=label_vmin, vmax=label_vmax)
axes[1, 0].set_title("Label original")

# Colonnes suivantes : rotations
for i, angle in enumerate(ANGLES):
    raw_rot = rotate(raw, angle, axes=(2, 1), reshape=False, order=3, mode="reflect")
    label_rot = rotate(label, angle, axes=(2, 1), reshape=False, order=0, mode="reflect")

    axes[0, i + 1].imshow(raw_rot[mid_z], cmap="gray", vmin=raw_vmin, vmax=raw_vmax)
    axes[0, i + 1].set_title(f"Raw {angle}°")

    axes[1, i + 1].imshow(label_rot[mid_z], cmap="gray", vmin=label_vmin, vmax=label_vmax)
    axes[1, i + 1].set_title(f"Label {angle}°")

    # Vérifications
    label_vals = np.unique(label_rot)
    print(f"Angle {angle:+3d}° | Label unique values (count): {len(label_vals)}, range: [{label_vals.min():.4f}, {label_vals.max():.4f}]")

for ax in axes.flat:
    ax.axis("off")

fig.suptitle(f"Vérification rotation en Z — coupe Z={mid_z}", fontsize=14)
plt.tight_layout()
plt.savefig(OUTPUT_PATH, dpi=150, bbox_inches="tight")
print(f"\nFigure sauvegardée: {OUTPUT_PATH}")

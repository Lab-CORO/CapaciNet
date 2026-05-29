# CapaciNet – 3D‑U‑Net for Robot Reachability Maps

> **Branch:** `vae_test/unet_3d`

CapaciNet provides a complete pipeline for generating robot **R**eachability **M**aps (RMs), formatting them as voxel grids, and training a 3‑D‑U‑Net to predict reachability in unseen environments. The code is written for **ROS 2 Humble** and **Python 3.10**. A CUDA‑enabled GPU is strongly recommended for training.

---

## 📦 Project layout

```
CapaciNet/
 └─ unet_3d/
     ├─ launch/                        # ROS 2 launch files
     │    ├─ create_reachability_map.launch.py
     │    └─ generate_data.launch.py
     ├─ script/
     │    ├─ format_data.py            # HDF5 converter: upsampling + augmentation + train/val split
     │    └─ augment_rotation_z.py     # RotationAugmenter class (used by format_data.py)
     ├─ config/
     │    └─ train_reach.yaml          # Training hyper‑parameters
     └─ data/
          ├─ train/                    # Raw + label volumes for training
          └─ val/                      # Raw + label volumes for validation
```

---

## 🚀 Quick start

### 1  Generate the reachability map

```bash
ros2 launch unet_3d create_reachability_map.launch.py
```

This step spawns your robot in a head‑less Gazebo instance and saves a coloured `.ply` RM in `~/.ros/`. Adjust the robot URDF or TCP frame as needed.

### 2  Create the raw dataset

```bash
ros2 launch unet_3d generate_data.launch.py output:=${PWD}/unet_3d/data/raw
```

Each RM voxel is exported as a 3‑D Numpy array together with a binary reachability label.

### 3  Re‑format the data

`unet_3d/script/format_data.py` reads the raw HDF5 files produced by the data generation step
(structure `group/N/voxel_grid` + `reachability_map`), converts them to the **pytorch‑3dunet**
layout (`raw` / `label` datasets), and optionally upsamples, augments, and splits the dataset.

**Context:** each source file can contain multiple robot configurations (groups). The script
iterates over all groups, automatically detects the voxel resolution from the `voxel_size`
attribute, and upsamples to 0.02 m if the resolution differs.

```bash
# Minimal usage – convert only
python unet_3d/script/format_data.py

# Custom input / output directories
python unet_3d/script/format_data.py \
    --input_dir  /path/to/raw/data/ \
    --output_dir /path/to/unet3d/data/

# With data augmentation (3 random Z‑rotations per group)
python unet_3d/script/format_data.py --n_copies 3

# With augmentation + train/val split (90 % / 10 %)
python unet_3d/script/format_data.py --n_copies 3 --split

# Full control
python unet_3d/script/format_data.py \
    --input_dir      /path/to/raw/data/ \
    --output_dir     /path/to/unet3d/data/ \
    --n_copies       5   \
    --angle_spectrum 90  \
    --aug_seed       0   \
    --split              \
    --val_ratio      0.15 \
    --seed           42
```

| Argument | Default | Description |
|---|---|---|
| `--input_dir` | `…/data/` | Source HDF5 files |
| `--output_dir` | `…/unet_3d/data/` | Destination directory |
| `--n_copies` | `0` | Augmented copies per group (0 = disabled) |
| `--angle_spectrum` | `180` | Max Z‑rotation angle in degrees |
| `--aug_seed` | `42` | Random seed for augmentation |
| `--split` | off | Move files into `train/` and `val/` after conversion |
| `--val_ratio` | `0.1` | Fraction of files assigned to validation |
| `--seed` | `0` | Random seed for train/val split |

The augmentation logic lives in `augment_rotation_z.py` as a `RotationAugmenter` class and
can also be imported independently in other scripts.

### 6  *(Optional)* Pad the volumes

If your data dimensions are **not** divisible by the network’s stride (e.g. 16), pad them:

```bash
python unet_3d/scripts/format_data.py --pad 8
```

### 7  Launch training

```bash
train3dunet --config unet_3d/config/train_reach.yaml
```

Monitor progress with **TensorBoard**:

```bash
tensorboard --logdir runs
```

---

## ⚙️ Environment

```bash
# Example using Miniconda
conda create -n capacinet python=3.10
conda activate capacinet
pip install -r requirements.txt  # See root folder

# ROS 2 Humble (Ubuntu 22.04)
source /opt/ros/humble/setup.bash
colcon build --packages-select unet_3d
```

Key dependencies:

* ROS 2 `sensor_msgs`, `open3d_ros2`
* `PyTorch >=2.0` + CUDA 11.8
* `pytorch‑3dunet`
* `open3d`, `h5py`, `scikit‑learn`

Docker images are provided in `docker/` for reproducibility.

---

## 📊 Results & checkpoints

Pre‑trained weights and sample RMs are available on the [release page](https://github.com/Lab-CORO/CapaciNet/releases). To evaluate on your own reachability map:

```bash
python unet_3d/scripts/infer.py --weights checkpoints/best_model.pth --input my_map.h5
```

---

## 🤝 Contributing

Issues and PRs are welcome! Please follow the [Conventional Commits](https://www.conventionalcommits.org) style and run `pre‑commit` before submitting.

---

## 📄 License

Distributed under the **MIT License**. See `LICENSE` for details.

---

## 📧 Contact

For questions, open an [issue](https://github.com/Lab-CORO/CapaciNet/issues) or reach us at **coro‑[dev@etsmtl.ca](mailto:dev@etsmtl.ca)**.

---

*Happy training!*

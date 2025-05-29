# CapaciNet – 3D‑U‑Net for Robot Reachability Maps

> **Branch:** `vae_test/unet_3d`

CapaciNet provides a complete pipeline for generating robot **R**eachability **M**aps (RMs), formatting them as voxel grids, and training a 3‑D‑U‑Net to predict reachability in unseen environments. The code is written for **ROS 2 Humble** and **Python 3.10**. A CUDA‑enabled GPU is strongly recommended for training.

---

## 📦 Project layout

```
CapaciNet/
 └─ unet_3d/
     ├─ launch/                  # ROS 2 launch files
     │    ├─ create_reachability_map.launch.py
     │    └─ generate_data.launch.py
     ├─ scripts/
     │    ├─ format_data.py      # HDF5 → Numpy/PNG converter
     │    └─ convert_to_unet.py  # Voxel grid → 3‑D‑U‑Net dataset
     ├─ config/
     │    └─ train_reach.yaml    # Training hyper‑parameters
     └─ data/
          ├─ train/              # Raw + label volumes for training
          └─ val/                # Raw + label volumes for validation
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

```bash
python unet_3d/scripts/format_data.py --input data/raw --output data/formatted
```

This converts the Numpy blobs to `*.h5` volumes and stores meta‑data such as voxel resolution, grid origin, and RM size as HDF5 attributes.

### 4  Convert to the 3‑D‑U‑Net layout

```bash
python unet_3d/scripts/convert_to_unet.py --src data/formatted --dst data
```

The script produces the folder hierarchy expected by **pytorch‑3dunet**.

### 5  Split into training and validation sets

```bash
python - <<'PY'
from pathlib import Path, shutil
from sklearn.model_selection import train_test_split
files = sorted(Path('unet_3d/data/raw').glob('*.h5'))
train, val = train_test_split(files, test_size=0.2, random_state=42)
for f in train: (Path('unet_3d/data/train')/f.name).write_bytes(f.read_bytes())
for f in val:   (Path('unet_3d/data/val')/f.name).write_bytes(f.read_bytes())
PY
```

Feel free to use a different split ratio.

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

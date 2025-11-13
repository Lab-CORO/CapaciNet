# CapaciNet - Complete User Guide

## Table of Contents

1. [Introduction](#introduction)
2. [Project Architecture](#project-architecture)
3. [Installation and Setup](#installation-and-setup)
4. [Dataset Generation](#dataset-generation)
5. [UNet3D Model Training](#unet3d-model-training)
6. [Using the Model in ROS2](#using-the-model-in-ros2)
7. [Configuration Parameters](#configuration-parameters)
8. [Troubleshooting](#troubleshooting)

---

## Introduction

**CapaciNet** is a complete pipeline for generating **Reachability Maps (RM)** for robotic manipulators, formatting them as voxelized grids, and training a 3D U-Net neural network to predict reachability in unseen environments.

### Main Features

- Automatic generation of reachability maps with obstacles
- Deep learning model training for reachability prediction
- ROS2 integration for real-time inference
- Docker support for maximum reproducibility

### System Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble
- **Python**: 3.10
- **GPU**: CUDA 11.8+ (strongly recommended for training)
- **RAM**: Minimum 16 GB
- **Disk Space**: ~50 GB for datasets

---

## Project Architecture

```
CapaciNet/
├── data_generation/          # Data generation and reachability maps
│   ├── src/                  # C++ source code for RM generation
│   ├── include/              # Header files
│   ├── launch/               # ROS2 launch files
│   │   ├── create_reachability_map.launch.py
│   │   └── generate_data.launch.py
│   ├── script/               # Python utilities
│   ├── data/                 # Output directory for generated data
│   └── docker/               # Docker configuration
│
└── unet_3d/                  # 3D U-Net training and inference
    ├── ros2_ws/              # ROS2 workspace
    │   └── src/capacitynet/  # ROS2 package for inference
    ├── script/               # Data formatting scripts
    ├── config/               # Training configurations
    │   ├── train_reach.yaml
    │   └── test_reach.yaml
    ├── data/                 # Training data
    │   ├── train/
    │   └── val/
    └── Dockerfile            # Docker configuration
```

### Main Components

#### 1. Data Generation (C++)

- **create_reachability_map**: Creates the base reachability map
- **generate_data**: Generates pairs of (voxel grid, reachability map)
- **obstacle_adder**: Adds random obstacles to the scene
- **scene_manager**: Orchestrates dataset generation

#### 2. Training (Python)

- **format_data.py**: Formats HDF5 data for pytorch-3dunet
- **train3dunet**: Trains the UNet3D model
- **YAML Configurations**: Network architecture and hyperparameters

#### 3. ROS2 Integration (Python)

- **capacitynet.py**: ROS2 node for real-time inference
- **Services**: Communication with curobo for voxel grids
- **Publisher**: Publishes predictions as PointCloud2

---

## Installation and Setup

### Option 1: Native Installation

#### Step 1: Install ROS2 Humble

```bash
# Add ROS2 repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

#### Step 2: Install C++ Dependencies

```bash
# Dependencies for data_generation
sudo apt install -y \
  liboctomap-dev \
  libeigen3-dev \
  libhdf5-dev \
  libopen3d-dev \
  ros-humble-pcl-ros \
  ros-humble-tf2-geometry-msgs
```

#### Step 3: Setup Python Environment

```bash
# Create conda environment
conda create -n capacinet python=3.10
conda activate capacinet

# Install PyTorch with CUDA
conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia

# Install Python dependencies
pip install pytorch-3dunet h5py scikit-learn open3d tensorboard
```

#### Step 4: Build ROS2 Workspace

```bash
cd CapaciNet
source /opt/ros/humble/setup.bash

# Build data_generation
cd data_generation
colcon build
source install/setup.bash

# Build ROS2 inference node
cd ../unet_3d/ros2_ws
colcon build --packages-select capacitynet
source install/setup.bash
```

### Option 2: Using Docker (Recommended)

#### For Data Generation

```bash
cd CapaciNet/data_generation/docker
docker build -t capacinet-datagen .

docker run --name datagen -it --gpus all --network=host \
  -v $PWD/../data:/workspace/data \
  capacinet-datagen
```

#### For Training and Inference

```bash
cd CapaciNet/unet_3d
docker build -f Dockerfile -t capacitynet .

# Launch container
./start.bash  # or:
docker run --name capacitynet2 -it --gpus all --network=host \
  --env DISPLAY=$DISPLAY \
  -v $PWD:/workspace/capacitynet \
  capacitynet
```

---

## Dataset Generation

The dataset generation process occurs in two main phases:

### Phase 1: Creating the Base Reachability Map

This step generates a complete reachability map for your robot in an empty environment.

#### Launch

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
cd CapaciNet/data_generation
source install/setup.bash

# Launch reachability map creation
ros2 launch data_generation create_reachability_map.launch.py
```

#### Configurable Parameters

You can modify parameters in the launch file or via command line:

```bash
ros2 launch data_generation create_reachability_map.launch.py \
  voxel_size:=0.08 \
  batch_size:=1000
```

**Available Parameters:**

- `voxel_size` (default: 0.08): Grid resolution in meters
- `batch_size` (default: 1000): Number of IK queries per batch
- `reach_max` (default: 1.3): Maximum workspace radius in meters

#### Detailed Process

1. **Workspace Discretization**
   - Creates a 3D grid around the robot base
   - Typical resolution: 0.08m (~38³ voxels for 3m space)

2. **Pose Generation**
   - For each voxel, generates 50 poses on a sphere
   - Poses are uniformly distributed (Fibonacci sampling)

3. **Reachability Computation**
   - Calls curobo IK service for each pose
   - Computes reachability score (0-1) based on valid IK solutions

4. **Saving**
   - Format: HDF5 + NPZ
   - Location: `~/.ros/master_ik_data0.08.h5`
   - Contains: voxel positions, reachability scores, metadata

#### Estimated Duration

- Small robot (6 DOF): ~30 minutes - 1 hour
- Large robot (7 DOF): ~1-2 hours
- Heavily depends on chosen resolution

### Phase 2: Generating Training Dataset

This phase generates multiple scenes with obstacles and computes corresponding reachability maps.

#### Launch

```bash
# In the same terminal as before
ros2 launch data_generation generate_data.launch.py \
  dataset_size:=100 \
  voxel_size:=0.02 \
  obj_max:=20
```

#### Configurable Parameters

**Dataset Parameters:**

- `dataset_size` (default: 5): Number of scenes to generate
- `voxel_size` (default: 0.02): Resolution for training (finer)
- `obj_max` (default: 20): Maximum obstacles per scene

**Obstacle Parameters:**

- Minimum size: 0.05m
- Maximum size: 0.3m
- Shape: cubes
- Position: random within workspace
- Collision validation with robot

#### Detailed Process

1. **Initialization**
   - Loads base reachability map
   - Initializes ROS2 services (curobo_ik, get_voxel_grid)

2. **For each scene (iteration):**

   a. **Scene Generation** (obstacle_adder)
      - Generates 3 to `obj_max` random cubes
      - Validates no obstacle collides with robot
      - Adds obstacles to simulation

   b. **Voxel Grid Acquisition** (generate_data)
      - Calls `/curobo_gen_traj/get_voxel_grid` service
      - Retrieves 3D occupancy grid (0=occupied, 1=free)

   c. **Reachability Computation**
      - Uses base map
      - Filters voxels colliding with obstacles
      - Computes adjusted reachability scores

   d. **Saving**
      - Format: HDF5 with group structure
      - Location: `data_generation/data/[timestamp].h5`

3. **Cleanup**
   - Removes obstacles from scene
   - Prepares next iteration

#### Generated Data Structure

```python
# HDF5 file: data_generation/data/scene_001.h5
/group/0/
├── voxel_grid        # 3D array (N×N×N): occupancy grid
│                     # 0 = voxel occupied by obstacle
│                     # 1 = voxel free
└── reachability_map  # 3D array (N×N×N): reachability scores
                      # 0.0 = unreachable
                      # 1.0 = fully reachable
```

#### Estimated Duration

- Per scene: ~30 seconds - 2 minutes
- 100-scene dataset: ~1-3 hours
- Depends on number of obstacles and resolution

#### Tips for Good Dataset

1. **Obstacle Diversity**
   - Use high `obj_max` (15-20)
   - Creates scenes with varying complexity

2. **Dataset Size**
   - Minimum recommended: 100 scenes
   - Optimal: 500-1000 scenes
   - More data = better generalization

3. **Resolution**
   - Training: 0.02m (fine)
   - Allows precise prediction
   - Trade-off between accuracy and computation speed

4. **Validation**
   - Visually check some scenes with `reachability_map_viz`
   - Ensure data is consistent

### Verify Generated Data

```bash
# List generated files
ls -lh data_generation/data/*.h5

# Visualize a reachability map (requires graphical display)
ros2 run data_generation reachability_map_viz \
  --input data_generation/data/scene_001.h5
```

---

## UNet3D Model Training

Once the dataset is generated, you can proceed with deep learning model training.

### Step 1: Data Formatting

Data must be converted to the format expected by `pytorch-3dunet`.

#### Formatting Script

```bash
cd CapaciNet
python unet_3d/script/format_data.py \
  --input data_generation/data \
  --output unet_3d/data/formatted
```

#### Script Options

```bash
python unet_3d/script/format_data.py \
  --input <source_directory> \
  --output <destination_directory> \
  --upsample 4  # Upsampling factor (optional)
```

**What does this script do?**

1. **Read source HDF5 files**
   - Loads `voxel_grid` and `reachability_map` from each group

2. **Upsampling (if needed)**
   - If base data has 0.08m resolution
   - Upsamples to 0.02m for training
   - Method: trilinear interpolation

3. **Format Conversion**
   - Saves in pytorch-3dunet format:
     - `/raw`: input voxel grid (obstacles)
     - `/label`: target reachability map (labels)
   - Compression: gzip (level 4)

4. **Metadata**
   - Resolution, grid origin, dimensions

#### Verify Formatting

```python
import h5py
import numpy as np

# Open formatted file
with h5py.File('unet_3d/data/formatted/sample_001.h5', 'r') as f:
    raw = f['raw'][:]      # Input grid
    label = f['label'][:]  # Target map

    print(f"Shape: {raw.shape}")
    print(f"Input range: [{raw.min()}, {raw.max()}]")
    print(f"Label range: [{label.min()}, {label.max()}]")
```

### Step 2: Train/Validation Split

Separate your dataset into training and validation sets.

#### Automatic Python Script

```bash
cd CapaciNet
python - <<'EOF'
from pathlib import Path
import shutil
from sklearn.model_selection import train_test_split

# List all files
data_dir = Path('unet_3d/data/formatted')
files = sorted(data_dir.glob('*.h5'))

print(f"Total files: {len(files)}")

# 80/20 split
train_files, val_files = train_test_split(
    files,
    test_size=0.2,
    random_state=42
)

# Create directories
train_dir = Path('unet_3d/data/train')
val_dir = Path('unet_3d/data/val')
train_dir.mkdir(parents=True, exist_ok=True)
val_dir.mkdir(parents=True, exist_ok=True)

# Copy files
for f in train_files:
    shutil.copy2(f, train_dir / f.name)

for f in val_files:
    shutil.copy2(f, val_dir / f.name)

print(f"Train: {len(train_files)} files")
print(f"Validation: {len(val_files)} files")
EOF
```

#### Manual Split

```bash
# Create directories
mkdir -p unet_3d/data/train unet_3d/data/val

# Manually move files
# 80% to train/, 20% to val/
```

### Step 3: Training Configuration

The configuration file controls all aspects of training.

#### File: `unet_3d/config/train_reach.yaml`

```yaml
# Model configuration
model:
  name: UNet3D
  in_channels: 1           # Voxel grid (1 channel = grayscale)
  out_channels: 1          # Reachability map (1 channel)
  f_maps: [16, 32, 64, 128, 256]  # Feature maps per level
  final_sigmoid: true      # Final activation for [0,1] scores
  num_groups: 8            # GroupNorm groups
  layer_order: gcr         # Group norm + Conv + ReLU
  num_levels: 5            # U-Net depth

# Loss function
loss:
  name: BCEDiceLoss       # Binary Cross-Entropy + Dice
  skip_last_target: false

# Optimizer
optimizer:
  name: Adam
  learning_rate: 0.0001   # 1e-4
  weight_decay: 0.00001   # 1e-5 (L2 regularization)

# Learning rate scheduler
lr_scheduler:
  name: ReduceLROnPlateau
  mode: min
  factor: 0.5             # Divide LR by 2
  patience: 10            # After 10 epochs without improvement
  min_lr: 0.0000001       # Minimum LR (1e-7)

# Training parameters
trainer:
  checkpoint_dir: unet_3d/checkpoints
  epochs: 500
  validate_after_iters: 10  # Validate every 10 iterations
  log_after_iters: 5        # Log every 5 iterations
  eval_score_higher_is_better: false  # Loss (lower = better)

# Data loader
loaders:
  train:
    file_paths:
      - unet_3d/data/train
    slice_builder:
      name: SliceBuilder
      patch_shape: [152, 152, 152]  # Patch size
      stride_shape: [76, 76, 76]    # 50% overlap
    transformer:
      raw:
        - name: Standardize         # Z-score normalization
        - name: RandomFlip          # Augmentation: random flip
        - name: RandomRotate90      # Augmentation: 90° rotation
        - name: ToTensor
          expand_dims: true
      label:
        - name: RandomFlip
        - name: RandomRotate90
        - name: ToTensor
          expand_dims: true
    batch_size: 1
    shuffle: true
    num_workers: 4

  val:
    file_paths:
      - unet_3d/data/val
    slice_builder:
      name: SliceBuilder
      patch_shape: [152, 152, 152]
      stride_shape: [152, 152, 152]  # No overlap for validation
    transformer:
      raw:
        - name: Standardize
        - name: ToTensor
          expand_dims: true
      label:
        - name: ToTensor
          expand_dims: true
    batch_size: 1
    shuffle: false
    num_workers: 4
```

#### Key Parameters to Adjust

**Model Architecture:**
- `f_maps`: Increase for more capacity (watch RAM usage)
- `num_levels`: Network depth (5 levels = good trade-off)

**Training:**
- `learning_rate`: Start at 1e-4, adjust if needed
- `batch_size`: Limited by GPU RAM (typically 1 for 3D volumes)
- `patch_shape`: Must be divisible by 2^(num_levels-1)

**Data Augmentation:**
- `RandomFlip`: Random flip on each axis
- `RandomRotate90`: Rotation by multiples of 90°
- Adjust based on generalization needs

### Step 4: Launch Training

#### With GPU (Recommended)

```bash
# Activate conda environment
conda activate capacinet

# Launch training
cd CapaciNet
train3dunet --config unet_3d/config/train_reach.yaml
```

#### Monitoring with TensorBoard

In a separate terminal:

```bash
conda activate capacinet
cd CapaciNet
tensorboard --logdir unet_3d/checkpoints --port 6006
```

Open browser: `http://localhost:6006`

**Metrics to Monitor:**

- **Training loss**: Should decrease steadily
- **Validation loss**: Should follow training loss
- **Learning rate**: Decreases automatically with scheduler
- **Time per epoch**: Depends on GPU and dataset size

#### Using Docker

```bash
# Launch container
cd CapaciNet/unet_3d
docker run --name training -it --gpus all \
  -v $PWD:/workspace/capacitynet \
  -p 6006:6006 \
  capacitynet bash

# Inside container
cd /workspace/capacitynet
train3dunet --config config/train_reach.yaml
```

### Step 5: Checkpoint Management

The model automatically saves checkpoints during training.

#### Checkpoint Types

```
unet_3d/checkpoints/
├── last_checkpoint.pytorch     # Last state (resume training)
├── best_checkpoint.pytorch     # Best model (minimum validation loss)
└── checkpoint_epoch_XXX.pytorch  # Intermediate checkpoints
```

#### Resume Interrupted Training

Modify `train_reach.yaml`:

```yaml
trainer:
  resume: unet_3d/checkpoints/last_checkpoint.pytorch
  checkpoint_dir: unet_3d/checkpoints
  # ... rest of configuration
```

Then relaunch:

```bash
train3dunet --config unet_3d/config/train_reach.yaml
```

### Step 6: Model Evaluation

#### Test Configuration

File: `unet_3d/config/test_reach.yaml`

```yaml
model:
  name: UNet3D
  in_channels: 1
  out_channels: 1
  f_maps: [16, 32, 64, 128, 256]
  final_sigmoid: true
  num_groups: 8
  layer_order: gcr
  num_levels: 5

# Load best checkpoint
model_path: unet_3d/checkpoints/best_checkpoint.pytorch

# Test data
loaders:
  test:
    file_paths:
      - unet_3d/data/val  # or separate test set
    slice_builder:
      name: SliceBuilder
      patch_shape: [152, 152, 152]
      stride_shape: [76, 76, 76]
    transformer:
      raw:
        - name: Standardize
        - name: ToTensor
          expand_dims: true
      label:
        - name: ToTensor
          expand_dims: true
```

#### Run Evaluation

```bash
predict3dunet --config unet_3d/config/test_reach.yaml
```

#### Visualize Results

Use the visualization script:

```bash
python unet_3d/script/format_vizualyze.py \
  --predictions unet_3d/predictions \
  --ground_truth unet_3d/data/val \
  --output unet_3d/visualizations
```

### Training Tips

#### Expected Performance

- **Final loss**: ~0.01 - 0.05 (BCE+Dice)
- **Convergence**: Typically 100-300 epochs
- **Training time**:
  - 100 scenes: ~2-4 hours (RTX 3080 GPU)
  - 500 scenes: ~10-20 hours

#### Signs of Problems

1. **Loss not decreasing**
   - Learning rate too high → reduce to 1e-5
   - Learning rate too low → increase to 5e-4
   - Data badly formatted → check preprocessing

2. **Overfitting** (train loss << val loss)
   - Increase dropout (add to config)
   - Increase data augmentation
   - Reduce model complexity

3. **Underfitting** (high train loss)
   - Increase `f_maps` (e.g., [32, 64, 128, 256, 512])
   - Increase `num_levels`
   - Train longer

4. **GPU memory issues**
   - Reduce `patch_shape` (e.g., [128, 128, 128])
   - Use `batch_size: 1`
   - Enable gradient checkpointing

---

## Using the Model in ROS2

Once the model is trained, you can use it for real-time inference in ROS2.

### ROS2 Node Architecture

The `capacitynet` node performs the following tasks:

1. **Service Client**: Retrieves voxel grid from curobo
2. **Inference**: Runs UNet3D model
3. **Publishing**: Publishes prediction as PointCloud2

### Step 1: Prepare Checkpoint

Copy your best checkpoint to the expected location:

```bash
mkdir -p unet_3d/ros2_ws/src/capacitynet/checkpoints
cp unet_3d/checkpoints/best_checkpoint.pytorch \
   unet_3d/ros2_ws/src/capacitynet/checkpoints/last_checkpoint.pytorch
```

### Step 2: Node Configuration

The node is configured via ROS2 parameters.

#### File: `unet_3d/ros2_ws/src/capacitynet/capacitynet/capacitynet.py`

**Configurable Parameters:**

```python
# Paths
MODEL_PATH = '/workspace/capacitynet/checkpoints/last_checkpoint.pytorch'
CONFIG_PATH = '/workspace/capacitynet/config/test_reach.yaml'

# Thresholds
REACHABILITY_THRESHOLD = 0.5  # Confidence threshold (0-1)

# Frequencies
INFERENCE_RATE = 0.5  # Hz (every 2 seconds)
PUBLISH_RATE = 1.0    # Hz (once per second)
```

#### Modify Parameters

Edit the `capacitynet.py` file:

```python
class ReachabilityNode(Node):
    def __init__(self):
        super().__init__('reachability_predictor')

        # Declare parameters
        self.declare_parameter('model_path', MODEL_PATH)
        self.declare_parameter('threshold', REACHABILITY_THRESHOLD)
        self.declare_parameter('inference_rate', INFERENCE_RATE)

        # Get values
        self.model_path = self.get_parameter('model_path').value
        self.threshold = self.get_parameter('threshold').value
        # ...
```

### Step 3: Build ROS2 Package

```bash
cd CapaciNet/unet_3d/ros2_ws
source /opt/ros/humble/setup.bash

# Build
colcon build --packages-select capacitynet

# Source workspace
source install/setup.bash
```

### Step 4: Launch Node

#### Option A: Direct Launch

```bash
# Terminal 1: Ensure curobo is running
ros2 run curobo_ros curobo_node

# Terminal 2: Launch prediction node
source /opt/ros/humble/setup.bash
cd CapaciNet/unet_3d/ros2_ws
source install/setup.bash

ros2 run capacitynet talker
```

#### Option B: With Custom Parameters

```bash
ros2 run capacitynet talker \
  --ros-args \
  -p model_path:=/path/to/checkpoint.pytorch \
  -p threshold:=0.6 \
  -p inference_rate:=1.0
```

#### Option C: With Docker (Recommended for Deployment)

```bash
# Launch container
cd CapaciNet/unet_3d
./start.bash

# Inside container
source /opt/ros/humble/setup.bash
cd /workspace/capacitynet/ros2_ws
source install/setup.bash
ros2 run capacitynet talker
```

### Step 5: Interact with Node

#### Published Topics

```bash
# View topics
ros2 topic list

# Main topic
/reachability_map_pc  # PointCloud2 (predicted reachability map)
```

#### Visualization in RViz2

```bash
# Launch RViz2
rviz2

# Configuration:
# 1. Fixed Frame: "world" or "base_link"
# 2. Add → PointCloud2
# 3. Topic: /reachability_map_pc
# 4. Color Transformer: "Intensity" or "RGB8"
```

**PointCloud Interpretation:**
- **Green/bright**: Highly reachable zones (score > 0.7)
- **Yellow**: Moderately reachable zones (0.4 - 0.7)
- **Red/dark**: Poorly reachable zones (< 0.4)
- **Absent**: Unreachable (score < threshold)

#### Used Services

The node calls curobo service:

```bash
# Check service is available
ros2 service list | grep voxel

# Test service manually
ros2 service call /curobo_gen_traj/get_voxel_grid \
  curobo_msgs/srv/GetVoxelGrid "{}"
```

#### Performance Monitoring

```bash
# View logs
ros2 run rqt_console rqt_console

# Monitor publishing frequency
ros2 topic hz /reachability_map_pc

# Monitor bandwidth
ros2 topic bw /reachability_map_pc
```

### Step 6: Integration in Your Application

#### Python Subscriber

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class ReachabilityConsumer(Node):
    def __init__(self):
        super().__init__('reachability_consumer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/reachability_map_pc',
            self.reachability_callback,
            10
        )

    def reachability_callback(self, msg):
        # Extract points from PointCloud2
        points = pc2.read_points(msg, field_names=['x', 'y', 'z', 'intensity'])

        for point in points:
            x, y, z, score = point
            print(f"Voxel ({x:.2f}, {y:.2f}, {z:.2f}): score = {score:.2f}")

        # Use reachability map for planning
        # ...

def main():
    rclpy.init()
    node = ReachabilityConsumer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### C++ Subscriber

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ReachabilityConsumer : public rclcpp::Node {
public:
    ReachabilityConsumer() : Node("reachability_consumer") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/reachability_map_pc", 10,
            std::bind(&ReachabilityConsumer::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        // Process points
        for (const auto& point : cloud->points) {
            RCLCPP_INFO(this->get_logger(),
                "Voxel (%.2f, %.2f, %.2f): score = %.2f",
                point.x, point.y, point.z, point.intensity
            );
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReachabilityConsumer>());
    rclcpp::shutdown();
    return 0;
}
```

### Step 7: Performance Optimization

#### Accelerate Inference

1. **TensorRT** (NVIDIA optimization)

```python
import torch
from torch2trt import torch2trt

# Convert PyTorch model to TensorRT
model_trt = torch2trt(
    model,
    [torch.randn(1, 1, 152, 152, 152).cuda()],
    fp16_mode=True  # Reduced precision
)

# Use in ROS2 node
torch.save(model_trt.state_dict(), 'model_trt.pth')
```

2. **Quantization**

```python
# Post-training quantization
quantized_model = torch.quantization.quantize_dynamic(
    model,
    {torch.nn.Linear, torch.nn.Conv3d},
    dtype=torch.qint8
)
```

3. **Batch Inference**

Modify node to process multiple grids simultaneously if your application allows.

#### Reduce Latency

```python
# In capacitynet.py
class ReachabilityNode(Node):
    def __init__(self):
        # ...

        # Preload model
        self.model.eval()
        with torch.no_grad():
            # Warm-up
            dummy_input = torch.randn(1, 1, 152, 152, 152).to(self.device)
            _ = self.model(dummy_input)

        # Use CUDA streams for parallelism
        self.stream = torch.cuda.Stream()
```

### Step 8: ROS2 Launch File (Optional)

Create a launch file to start all necessary nodes:

#### File: `unet_3d/ros2_ws/src/capacitynet/launch/inference.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Parameters
        DeclareLaunchArgument(
            'model_path',
            default_value='/workspace/capacitynet/checkpoints/last_checkpoint.pytorch',
            description='Path to model checkpoint'
        ),
        DeclareLaunchArgument(
            'threshold',
            default_value='0.5',
            description='Reachability threshold (0-1)'
        ),

        # Prediction node
        Node(
            package='capacitynet',
            executable='talker',
            name='reachability_predictor',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'threshold': LaunchConfiguration('threshold'),
            }],
            output='screen'
        ),

        # RViz2 for visualization (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/workspace/capacitynet/config/reachability.rviz'],
            output='screen'
        ),
    ])
```

#### Usage

```bash
ros2 launch capacitynet inference.launch.py \
  model_path:=/path/to/model.pytorch \
  threshold:=0.6
```

---

## Configuration Parameters

### Summary of Key Parameters

#### Data Generation

| Parameter | Default | Description | Recommendation |
|-----------|---------|-------------|----------------|
| `voxel_size` | 0.02m | Grid resolution | 0.02m for precision |
| `dataset_size` | 5 | Number of scenes | 100-500 for good model |
| `batch_size` | 1000 | IK batch size | 1000-2000 based on GPU |
| `reach_max` | 1.3m | Workspace radius | Based on your robot |
| `obj_max` | 20 | Max obstacles/scene | 15-20 for diversity |

#### Training

| Parameter | Default | Description | Recommendation |
|-----------|---------|-------------|----------------|
| `learning_rate` | 1e-4 | Learning rate | 1e-4 to 1e-5 |
| `batch_size` | 1 | Images per batch | 1 (RAM limited) |
| `patch_shape` | [152³] | Patch size | Divisible by 16 |
| `epochs` | 500 | Number of epochs | 200-500 |
| `f_maps` | [16,32,64,128,256] | U-Net channels | Increase if needed |

#### ROS2 Inference

| Parameter | Default | Description | Recommendation |
|-----------|---------|-------------|----------------|
| `threshold` | 0.5 | Confidence threshold | 0.4-0.6 based on use |
| `inference_rate` | 0.5 Hz | Inference frequency | 0.5-2 Hz |
| `publish_rate` | 1.0 Hz | Publishing frequency | 1-5 Hz |

---

## Troubleshooting

### Common Problems and Solutions

#### 1. Data Generation

**Problem:** "Service /curobo_ik/ik_batch not available"
```bash
# Check curobo is running
ros2 service list | grep curobo

# Solution: Launch curobo
ros2 run curobo_ros curobo_node
```

**Problem:** "No IK solution found for any pose"
```bash
# Check robot configuration
# Increase reach_max or adjust joint limits
ros2 launch data_generation create_reachability_map.launch.py reach_max:=1.5
```

**Problem:** Empty or incomplete dataset
```bash
# Check disk space
df -h

# Check permissions
ls -la data_generation/data/

# Check logs
ros2 launch data_generation generate_data.launch.py 2>&1 | tee generation.log
```

#### 2. Training

**Problem:** "CUDA out of memory"
```yaml
# Reduce patch_shape in train_reach.yaml
patch_shape: [128, 128, 128]  # Instead of [152, 152, 152]

# Or reduce f_maps
f_maps: [16, 32, 64, 128]  # Instead of [16, 32, 64, 128, 256]
```

**Problem:** Loss = NaN
```yaml
# Reduce learning rate
optimizer:
  learning_rate: 0.00001  # 1e-5 instead of 1e-4

# Check data
python -c "
import h5py
import numpy as np
f = h5py.File('unet_3d/data/train/sample.h5', 'r')
print('Raw:', np.isnan(f['raw'][:]).any())
print('Label:', np.isnan(f['label'][:]).any())
"
```

**Problem:** Training very slow
```bash
# Check GPU usage
nvidia-smi

# Reduce num_workers if CPU saturated
num_workers: 2  # Instead of 4

# Use more powerful GPU or reduce resolution
```

#### 3. ROS2 Inference

**Problem:** "Model checkpoint not found"
```bash
# Check path
ls -la unet_3d/ros2_ws/src/capacitynet/checkpoints/

# Copy checkpoint
cp unet_3d/checkpoints/best_checkpoint.pytorch \
   unet_3d/ros2_ws/src/capacitynet/checkpoints/last_checkpoint.pytorch
```

**Problem:** "No PointCloud published"
```bash
# Check voxel service is available
ros2 service call /curobo_gen_traj/get_voxel_grid curobo_msgs/srv/GetVoxelGrid

# Check node logs
ros2 run capacitynet talker --ros-args --log-level debug
```

**Problem:** Very slow inference
```python
# Enable evaluation mode
model.eval()

# Disable gradients
with torch.no_grad():
    prediction = model(input_tensor)

# Check CUDA is used
print(f"Device: {next(model.parameters()).device}")
```

#### 4. Visualization

**Problem:** RViz doesn't display PointCloud
```bash
# Check topic
ros2 topic echo /reachability_map_pc --no-arr

# Check Fixed Frame in RViz
# Must match PointCloud frame_id

# Check format
ros2 topic info /reachability_map_pc
```

**Problem:** PointCloud is empty
```python
# Reduce threshold
threshold: 0.3  # Instead of 0.5

# Check model predicts correct values
# Add logs in capacitynet.py
```

### Logging and Debugging

#### Enable Detailed Logs

```bash
# ROS2 logs
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1

ros2 run capacitynet talker --ros-args --log-level debug
```

#### Check System Status

```bash
# GPU
nvidia-smi

# ROS2 topics
ros2 topic list
ros2 topic hz /reachability_map_pc

# Active nodes
ros2 node list

# Available services
ros2 service list
```

#### Performance Profiling

```python
# Add to capacitynet.py
import time

def inference_callback(self):
    start = time.time()

    # Service call
    t1 = time.time()
    voxel_grid = self.get_voxel_grid()
    self.get_logger().info(f"Voxel fetch: {(time.time()-t1)*1000:.1f}ms")

    # Inference
    t2 = time.time()
    prediction = self.predict(voxel_grid)
    self.get_logger().info(f"Inference: {(time.time()-t2)*1000:.1f}ms")

    # Total
    self.get_logger().info(f"Total: {(time.time()-start)*1000:.1f}ms")
```

---

## Appendices

### A. HDF5 File Structure

#### Generated data file

```
data_generation/data/scene_XXX.h5
├── /group/0/
│   ├── voxel_grid        # (N, N, N) float32
│   └── reachability_map  # (N, N, N) float32
├── /group/1/
│   ├── voxel_grid
│   └── reachability_map
└── ...
```

#### Formatted file for training

```
unet_3d/data/train/sample_XXX.h5
├── /raw           # (1, N, N, N) float32 - Voxel grid
├── /label         # (1, N, N, N) float32 - Reachability map
└── attributes:
    ├── resolution  # float (e.g., 0.02)
    ├── origin      # [x, y, z]
    └── shape       # [N, N, N]
```

### B. UNet3D Model Architecture

```
UNet3D(
  Encoder:
    Level 1: Conv3D(1→16) + GroupNorm + ReLU
    Level 2: Conv3D(16→32) + GroupNorm + ReLU + MaxPool3D
    Level 3: Conv3D(32→64) + GroupNorm + ReLU + MaxPool3D
    Level 4: Conv3D(64→128) + GroupNorm + ReLU + MaxPool3D
    Level 5: Conv3D(128→256) + GroupNorm + ReLU + MaxPool3D

  Decoder:
    Level 4: Upsample + Concat + Conv3D(256+128→128)
    Level 3: Upsample + Concat + Conv3D(128+64→64)
    Level 2: Upsample + Concat + Conv3D(64+32→32)
    Level 1: Upsample + Concat + Conv3D(32+16→16)

  Output: Conv3D(16→1) + Sigmoid
)

Total parameters: ~7.8M
```

### C. Curobo ROS2 Services

```bash
# IK batch
ros2 service call /curobo_ik/ik_batch curobo_msgs/srv/IkBatch \
  "{poses: [{position: {x: 0.5, y: 0.2, z: 0.3}, orientation: {w: 1.0}}]}"

# Get voxel grid
ros2 service call /curobo_gen_traj/get_voxel_grid curobo_msgs/srv/GetVoxelGrid

# Add object
ros2 service call /curobo_gen_traj/add_object curobo_msgs/srv/AddObject \
  "{name: 'cube1', type: 'cube', dimensions: [0.1, 0.1, 0.1], ...}"

# Remove object
ros2 service call /curobo_gen_traj/remove_object curobo_msgs/srv/RemoveObject \
  "{name: 'cube1'}"
```

### D. Complete Workflow Example

```bash
#!/bin/bash
# Complete script: generation → training → deployment

set -e  # Stop on error

echo "=== PHASE 1: Base map generation ==="
cd ~/CapaciNet/data_generation
source install/setup.bash
ros2 launch data_generation create_reachability_map.launch.py

echo "=== PHASE 2: Dataset generation (100 scenes) ==="
ros2 launch data_generation generate_data.launch.py dataset_size:=100

echo "=== PHASE 3: Data formatting ==="
cd ~/CapaciNet
conda activate capacinet
python unet_3d/script/format_data.py \
  --input data_generation/data \
  --output unet_3d/data/formatted

echo "=== PHASE 4: Train/val split ==="
python - <<'EOF'
from pathlib import Path
import shutil
from sklearn.model_selection import train_test_split

files = sorted(Path('unet_3d/data/formatted').glob('*.h5'))
train, val = train_test_split(files, test_size=0.2, random_state=42)

Path('unet_3d/data/train').mkdir(exist_ok=True)
Path('unet_3d/data/val').mkdir(exist_ok=True)

for f in train: shutil.copy2(f, Path('unet_3d/data/train')/f.name)
for f in val: shutil.copy2(f, Path('unet_3d/data/val')/f.name)

print(f"Train: {len(train)}, Val: {len(val)}")
EOF

echo "=== PHASE 5: Training ==="
train3dunet --config unet_3d/config/train_reach.yaml

echo "=== PHASE 6: Deployment preparation ==="
mkdir -p unet_3d/ros2_ws/src/capacitynet/checkpoints
cp unet_3d/checkpoints/best_checkpoint.pytorch \
   unet_3d/ros2_ws/src/capacitynet/checkpoints/last_checkpoint.pytorch

echo "=== PHASE 7: ROS2 build ==="
cd unet_3d/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select capacitynet

echo "=== COMPLETE! ==="
echo "To launch inference:"
echo "  source unet_3d/ros2_ws/install/setup.bash"
echo "  ros2 run capacitynet talker"
```

### E. Resources and References

#### Official Documentation

- **ROS2 Humble**: https://docs.ros.org/en/humble/
- **PyTorch**: https://pytorch.org/docs/stable/
- **pytorch-3dunet**: https://github.com/wolny/pytorch-3d-unet
- **curobo**: https://curobo.org/

#### Reference Papers

- **3D U-Net**: Çiçek et al., "3D U-Net: Learning Dense Volumetric Segmentation from Sparse Annotation" (2016)
- **Reachability Maps**: Zacharias et al., "The Capability Map: A Tool for Reachability Analysis" (2013)

#### Support

- **GitHub Issues**: https://github.com/Lab-CORO/CapaciNet/issues
- **Email**: coro-dev@etsmtl.ca

---

**Last updated**: 2025-01-13
**Version**: 1.0.0
**License**: MIT

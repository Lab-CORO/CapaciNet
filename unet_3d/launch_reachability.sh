#!/bin/bash
# Script de lancement pour le nœud reachability avec le bon environnement

# Activer conda
source /opt/conda/etc/profile.d/conda.sh

# Utiliser l'environnement ros2_torch (Python 3.10)
conda activate ros2_torch

# Sourcer ROS2
source /opt/ros/humble/setup.bash
source /workspace/capacitynet/ros2_ws/install/setup.bash

# Désactiver le profiling PyTorch pour éviter les bugs de timing
export CUDA_LAUNCH_BLOCKING=0
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512
export TORCH_USE_RTLD_GLOBAL=1

# Lancer le nœud directement avec le bon Python
echo "🚀 Lancement du nœud reachability..."
python /workspace/capacitynet/ros2_ws/src/capacitynet/capacitynet/capacitynet.py

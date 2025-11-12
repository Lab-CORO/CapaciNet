#!/bin/bash
# Solution 1: Créer un nouvel environnement conda avec Python 3.10 compatible ROS2

set -e

# Initialiser conda d'abord
echo "Initialisation de conda..."
if [ -f /opt/conda/etc/profile.d/conda.sh ]; then
    source /opt/conda/etc/profile.d/conda.sh
elif [ -f ~/anaconda3/etc/profile.d/conda.sh ]; then
    source ~/anaconda3/etc/profile.d/conda.sh
elif [ -f ~/miniconda3/etc/profile.d/conda.sh ]; then
    source ~/miniconda3/etc/profile.d/conda.sh
else
    echo "❌ Conda non trouvé. Veuillez l'installer d'abord."
    exit 1
fi

echo "Création d'un nouvel environnement conda 'ros2_torch' avec Python 3.10..."
conda create -n ros2_torch python=3.10 -y

echo "Activation de l'environnement..."
conda activate ros2_torch

echo "Installation de PyTorch et dépendances..."
# Installer PyTorch avec CUDA 12.4 support (compatible avec 12.9 via rétrocompatibilité)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124

echo "Installation de open3d et autres dépendances..."
pip install open3d pyyaml empy==3.3.4 lark catkin_pkg h5py scikit-image

echo "Installation de pytorch-3dunet (via conda, sans réinstaller les dépendances)..."
conda install -c conda-forge pytorch-3dunet --no-deps -y

echo "✅ Environnement ros2_torch créé avec succès!"
echo ""
echo "Pour l'utiliser :"
echo "  conda activate ros2_torch"
echo "  source /opt/ros/humble/setup.bash"
echo "  source /workspace/capacitynet/ros2_ws/install/setup.bash"
echo "  ros2 run capacitynet reachability_node"

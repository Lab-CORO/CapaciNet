# ============================================================================
# Image de base: ROS2 Humble avec Python 3.10 (meilleure compatibilité)
# ============================================================================
FROM osrf/ros:humble-desktop

# Éviter les prompts interactifs
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Montreal

# ============================================================================
# ÉTAPE 1: Installation des dépendances système
# ============================================================================

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    git \
    wget \
    curl \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Installer packages ROS2 supplémentaires
RUN apt-get update && apt-get install -y \
    ros-humble-nav2-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cyclonedds \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Configuration ROS2
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ============================================================================
# ÉTAPE 2: Installation de Miniconda
# ============================================================================

# Installer Miniconda
RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    /bin/bash ~/miniconda.sh -b -p /opt/conda && \
    rm ~/miniconda.sh && \
    /opt/conda/bin/conda clean --all -y && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh

# Configurer conda pour qu'il soit accessible
ENV PATH=/opt/conda/bin:$PATH

# Initialiser conda
RUN conda init bash

# Accepter les termes de service Anaconda
RUN conda config --set allow_conda_downgrades true && \
    conda config --set channel_priority flexible && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main && \
    conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

# ============================================================================
# ÉTAPE 3: Création de l'environnement conda 'ros2_torch'
# ============================================================================

# Créer un environnement conda dédié avec Python 3.10 (compatible ROS2)
RUN conda create -n ros2_torch python=3.10 -y

# Activer l'environnement et installer PyTorch avec CUDA 12.4 support
RUN /bin/bash -c "source /opt/conda/etc/profile.d/conda.sh && \
    conda activate ros2_torch && \
    pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cu124"

# ============================================================================
# ÉTAPE 4: Installation des dépendances ML et ROS2 Python
# ============================================================================

# Installer open3d et autres dépendances Python
RUN /bin/bash -c "source /opt/conda/etc/profile.d/conda.sh && \
    conda activate ros2_torch && \
    pip install --no-cache-dir \
        open3d \
        pyyaml \
        empy==3.3.4 \
        lark \
        catkin_pkg \
        h5py \
        scikit-image \
        tensorboard"

# Installer pytorch-3dunet via conda (sans dépendances pour éviter les conflits)
RUN /bin/bash -c "source /opt/conda/etc/profile.d/conda.sh && \
    conda activate ros2_torch && \
    conda install -c conda-forge pytorch-3dunet --no-deps -y && \
    conda clean -a -y"

# Configurer PYTHONPATH pour que ROS2 trouve les packages conda
ENV PYTHONPATH=/opt/conda/envs/ros2_torch/lib/python3.10/site-packages

# ============================================================================
# ÉTAPE 5: Setup du workspace ROS2
# ============================================================================

# Créer la structure du workspace
WORKDIR /workspace/capacitynet
RUN mkdir -p ros2_ws/src

# Cloner curobo_msgs
RUN cd ros2_ws/src && \
    git clone https://github.com/Lab-CORO/curobo_msgs.git

# Copier le code capacitynet (décommenter pour build)
# COPY ./ros2_ws/src/capacitynet /workspace/capacitynet/ros2_ws/src/capacitynet
# COPY ./config /workspace/capacitynet/config

# Compiler le workspace ROS2 (utiliser le Python système, pas celui de conda)
RUN bash -c "source /opt/ros/humble/setup.bash && \
    cd /workspace/capacitynet/ros2_ws && \
    PATH=/usr/bin:$PATH colcon build --symlink-install"

# ============================================================================
# ÉTAPE 6: Configuration de l'environnement
# ============================================================================

# Créer le script d'entrée
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Initialiser et activer conda\n\
source /opt/conda/etc/profile.d/conda.sh\n\
conda activate ros2_torch\n\
\n\
# Sourcer ROS2\n\
source /opt/ros/humble/setup.bash\n\
source /workspace/capacitynet/ros2_ws/install/setup.bash 2>/dev/null || true\n\
\n\
# Variables d'\''environnement pour PyTorch/CUDA\n\
export CUDA_LAUNCH_BLOCKING=0\n\
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512\n\
export TORCH_USE_RTLD_GLOBAL=1\n\
export PYTHONPATH=/opt/conda/envs/ros2_torch/lib/python3.10/site-packages:$PYTHONPATH\n\
\n\
# Afficher les informations\n\
echo "========================================"\n\
echo "🤖 CapacityNet ROS2 Environment"\n\
echo "========================================"\n\
echo "Conda Environment: ros2_torch"\n\
echo "ROS2 Distro: $(printenv ROS_DISTRO)"\n\
echo "Python: $(python --version)"\n\
echo "Conda: $(conda --version)"\n\
python -c "import torch; print(f'\''PyTorch: {torch.__version__}'\'')" 2>/dev/null || echo "PyTorch: Not loaded yet"\n\
python -c "import torch; print(f'\''CUDA Available: {torch.cuda.is_available()}'\'')" 2>/dev/null || true\n\
python -c "from pytorch3dunet.unet3d import UNet3D; print('\''✅ pytorch-3dunet importé avec succès'\'')" 2>/dev/null || echo "⚠️  pytorch-3dunet: erreur d'\''import"\n\
echo "========================================"\n\
echo ""\n\
\n\
exec "$@"\n\
' > /workspace/entrypoint.sh && chmod +x /workspace/entrypoint.sh

# Créer un script de lancement pour le nœud reachability
RUN echo '#!/bin/bash\n\
echo "🚀 Lancement du nœud CapacityNet Reachability..."\n\
python3 /workspace/capacitynet/ros2_ws/src/capacitynet/capacitynet/capacitynet.py\n\
' > /workspace/launch_node.sh && chmod +x /workspace/launch_node.sh

# Créer un script de recompilation rapide
RUN echo '#!/bin/bash\n\
echo "🔨 Recompilation du workspace ROS2..."\n\
cd /workspace/capacitynet/ros2_ws\n\
source /opt/ros/humble/setup.bash\n\
PATH=/usr/bin:$PATH colcon build --symlink-install\n\
echo "✅ Compilation terminée!"\n\
' > /workspace/rebuild.sh && chmod +x /workspace/rebuild.sh

# Configurer .bashrc pour sourcer automatiquement
RUN echo 'source /opt/conda/etc/profile.d/conda.sh' >> ~/.bashrc && \
    echo 'conda activate ros2_torch' >> ~/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /workspace/capacitynet/ros2_ws/install/setup.bash 2>/dev/null || true' >> ~/.bashrc && \
    echo 'export CUDA_LAUNCH_BLOCKING=0' >> ~/.bashrc && \
    echo 'export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512' >> ~/.bashrc && \
    echo 'export PYTHONPATH=/opt/conda/envs/ros2_torch/lib/python3.10/site-packages:$PYTHONPATH' >> ~/.bashrc && \
    echo 'export PS1="\[\e[32m\](ros2_torch)\[\e[0m\] \[\e[34m\]\w\[\e[0m\]\$ "' >> ~/.bashrc && \
    echo 'alias rebuild="/workspace/rebuild.sh"' >> ~/.bashrc && \
    echo 'alias launch="/workspace/launch_node.sh"' >> ~/.bashrc

# ============================================================================
# ÉTAPE 7: Configuration finale
# ============================================================================

WORKDIR /workspace/capacitynet

ENTRYPOINT ["/workspace/entrypoint.sh"]
CMD ["/bin/bash"]

# ============================================================================
# METADATA ET DOCUMENTATION
# ============================================================================

LABEL maintainer="Lab-CORO"
LABEL description="ROS2 Humble + PyTorch + CUDA - CapacityNet Reachability Node"
LABEL ros.distro="humble"
LABEL pytorch.version="2.6.0"
LABEL python.version="3.10"
LABEL cuda.version="12.4"


# ============================================================================
# INSTRUCTIONS D'UTILISATION
# ============================================================================

# 🏗️  CONSTRUCTION:
#   docker build -f Dockerfile -t capacitynet:latest .

# 🚀 LANCEMENT INTERACTIF:
#   docker run -it --gpus all \
#     -v $(pwd):/workspace/capacitynet \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     -e DISPLAY=$DISPLAY \
#     --network host \
#     capacitynet:latest

# 🤖 LANCER LE NŒUD DIRECTEMENT:
#   docker run --rm -it --gpus all \
#     -v $(pwd):/workspace/capacitynet \
#     --network host \
#     capacitynet:latest /workspace/launch_node.sh
#
# 🔨 RECOMPILER LE WORKSPACE (après modifications):
#   docker run --rm -it --gpus all \
#     -v $(pwd):/workspace/capacitynet \
#     capacitynet:latest /workspace/rebuild.sh
#
# 📊 TESTER L'ENVIRONNEMENT:
#   docker run --rm -it --gpus all capacitynet:latest \
#     python3 -c "import torch; import rclpy; print('✅ OK')"
#
# DANS LE CONTENEUR:
#   rebuild    # Recompiler le workspace
#   launch     # Lancer le nœud
#   ros2 run capacitynet reachability_node  # Alternative
#
# ============================================================================

FROM curobo_ros:x86-jazzy

# ── Base: curobo_ros:x86-jazzy (CUDA / Ubuntu 24.04 Noble / Python 3.12)
# Déjà disponible dans la base :
#   Python 3.12, PyTorch + CUDA (NVIDIA NGC), CuRobo, numpy 1.26.4,
#   ROS2 Jazzy + ros-jazzy-*, TensorRT (fourni par l'image NVIDIA — pas
#   besoin de le réinstaller comme sur Jetson/CSV)

LABEL maintainer="Lab-CORO"
LABEL description="CapacityNet Reachability Node — ROS2 Jazzy + PyTorch (x86)"
LABEL ros.distro="jazzy"
LABEL python.version="3.12"

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Montreal

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 1 — Dépendances Python CapacityNet
# open3d/numpy/scipy sont déjà dans la base — on installe uniquement ce qui manque.
# ═══════════════════════════════════════════════════════════════════════════════
RUN pip3 install --no-cache-dir \
    h5py \
    scikit-image \
    tensorboard \
    pyyaml \
    "empy==3.3.4" \
    lark \
    catkin_pkg

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 2 — pytorch-3dunet (installation depuis les sources)
#
# CRITIQUE : --no-deps est OBLIGATOIRE. pytorch-3dunet déclare `torch` et
# `torchvision` en dépendances ; sans --no-deps, pip télécharge un wheel torch
# générique de PyPI et ÉCRASE le PyTorch CUDA de la base image.
# Toutes les vraies deps sont déjà présentes :
#   torch (base) · numpy/scipy (base) · h5py/scikit-image/tensorboard (ÉTAPE 1)
#   torchvision : déclaré mais jamais importé par le code → non installé.
# ═══════════════════════════════════════════════════════════════════════════════
RUN git clone --depth=1 https://github.com/wolny/pytorch-3dunet.git /tmp/pytorch-3dunet && \
    cd /tmp/pytorch-3dunet && \
    SETUPTOOLS_SCM_PRETEND_VERSION=1.0.0 pip3 install --no-cache-dir --no-build-isolation --ignore-requires-python --no-deps . && \
    rm -rf /tmp/pytorch-3dunet

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 3 — Re-pin numpy<2
# pytorch-3dunet ou ses dépendances peuvent remonter numpy 2.x.
# La base curobo_ros:x86-jazzy pin numpy==1.26.4 (compat numba/cupy) — doit rester <2.
# ═══════════════════════════════════════════════════════════════════════════════
RUN pip3 install --no-cache-dir "numpy<2"

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 4 — Workspace ROS2 CapacityNet
# ═══════════════════════════════════════════════════════════════════════════════
RUN mkdir -p /workspace/capacitynet/ros2_ws/src
WORKDIR /workspace/capacitynet/ros2_ws/src

RUN git clone https://github.com/Lab-CORO/curobo_msgs.git

# reachability_map_visualizer : dépendance de capacitynet (package.xml).
# Requis pour le colcon build de capacitynet monté au runtime (mode DEV).
RUN git clone https://github.com/Lab-CORO/reachability_map_visualizer.git

# Deps système de reachability_map_visualizer (plugin RViz Qt + HDF5).
# rviz2/rviz_common/rviz_rendering/pluginlib viennent de ros-jazzy-desktop (base).
RUN apt-get update && apt-get install -y --no-install-recommends \
    qtbase5-dev libhdf5-dev \
    && rm -rf /var/lib/apt/lists/*

# Copier le code capacitynet (décommenter pour build)
# COPY ./ros2_ws/src/capacitynet /workspace/capacitynet/ros2_ws/src/capacitynet

WORKDIR /workspace/capacitynet/ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install"

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 4b — RMW : Cyclone DDS (remplace FastDDS)
# Meilleur comportement sur gros messages (PointCloud2 reachability_map) et config
# plus simple. DOIT être identique sur TOUT le graphe ROS (capacitynet + leeloo),
# sinon capacitynet ne reçoit plus /curobo_trajectory_planner/voxel_grid_sparse.
# ═══════════════════════════════════════════════════════════════════════════════
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 5 — Entrypoint (sans conda — Python 3.12 global hérité de la base)
# ═══════════════════════════════════════════════════════════════════════════════
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/jazzy/setup.bash\nsource /workspace/capacitynet/ros2_ws/install/setup.bash 2>/dev/null || true\nexport CUDA_LAUNCH_BLOCKING=0\nexport PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512\necho "========================================"\necho "CapacityNet ROS2 Environment (x86)"\necho "========================================"\necho "ROS2 Distro: $(printenv ROS_DISTRO)"\necho "Python: $(python3 --version)"\npython3 -c "import torch; print('"'"'PyTorch: '"'"' + torch.__version__)" 2>/dev/null || echo "PyTorch: not loaded"\npython3 -c "import torch; print('"'"'CUDA Available: '"'"' + str(torch.cuda.is_available()))" 2>/dev/null || true\npython3 -c "from pytorch3dunet.unet3d import model; print('"'"'pytorch-3dunet: OK'"'"')" 2>/dev/null || echo "pytorch-3dunet: import error"\necho "========================================"\nexec "$@"\n' > /workspace/entrypoint.sh && chmod +x /workspace/entrypoint.sh

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 6 — Scripts utilitaires
# ═══════════════════════════════════════════════════════════════════════════════
RUN printf '#!/bin/bash\necho "Recompilation du workspace ROS2..."\ncd /workspace/capacitynet/ros2_ws\nsource /opt/ros/jazzy/setup.bash\ncolcon build --symlink-install\necho "Compilation terminee!"\n' > /workspace/rebuild.sh && chmod +x /workspace/rebuild.sh

RUN printf '#!/bin/bash\necho "Lancement du noeud CapacityNet Reachability..."\npython3 /workspace/capacitynet/ros2_ws/src/capacitynet/capacitynet/capacitynet.py\n' > /workspace/launch_node.sh && chmod +x /workspace/launch_node.sh

# ═══════════════════════════════════════════════════════════════════════════════
# ÉTAPE 7 — .bashrc
# ═══════════════════════════════════════════════════════════════════════════════
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && \
    echo 'source /workspace/capacitynet/ros2_ws/install/setup.bash 2>/dev/null || true' >> ~/.bashrc && \
    echo 'export CUDA_LAUNCH_BLOCKING=0' >> ~/.bashrc && \
    echo 'export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512' >> ~/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc && \
    echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc && \
    echo 'export CYCLONEDDS_URI=/workspace/capacitynet/ros2_ws/src/capacitynet/config/profile.xml' >> ~/.bashrc && \
    echo 'alias rebuild="/workspace/rebuild.sh"' >> ~/.bashrc && \
    echo 'alias launch="/workspace/launch_node.sh"' >> ~/.bashrc

WORKDIR /workspace/capacitynet

ENTRYPOINT ["/workspace/entrypoint.sh"]
CMD ["/bin/bash"]

# ============================================================================
# UTILISATION
# ============================================================================

# 🏗️  PRÉREQUIS : builder l'image de base curobo_ros:x86-jazzy
#   docker build -f src/curobo_ros/docker/x86_jazzy.dockerfile \
#     -t curobo_ros:x86-jazzy .

# 🏗️  CONSTRUCTION:
#   docker build -f Dockerfile -t capacitynet:latest .

# 🚀 LANCEMENT INTERACTIF:
#   docker run -it --gpus all \
#     -v $(pwd)/ros2_ws/src/capacitynet:/workspace/capacitynet/ros2_ws/src/capacitynet \
#     --network host \
#     capacitynet:latest

# 🤖 LANCER LE NŒUD DIRECTEMENT:
#   docker run --rm -it --gpus all \
#     -v $(pwd)/ros2_ws/src/capacitynet:/workspace/capacitynet/ros2_ws/src/capacitynet \
#     --network host \
#     capacitynet:latest /workspace/launch_node.sh
#
# 🔨 RECOMPILER LE WORKSPACE (après modifications):
#   docker run --rm -it --gpus all \
#     -v $(pwd)/ros2_ws/src/capacitynet:/workspace/capacitynet/ros2_ws/src/capacitynet \
#     capacitynet:latest /workspace/rebuild.sh
#
# DANS LE CONTENEUR:
#   rebuild    # Recompiler le workspace
#   launch     # Lancer le nœud
#   ros2 run capacitynet reachability_node  # Alternative
#
# ============================================================================

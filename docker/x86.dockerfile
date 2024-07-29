##
## Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
##
## NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
## property and proprietary rights in and to this material, related
## documentation and any modifications thereto. Any use, reproduction,
## disclosure or distribution of this material and related documentation
## without an express license agreement from NVIDIA CORPORATION or
## its affiliates is strictly prohibited.
##
FROM nvcr.io/nvidia/pytorch:23.08-py3 AS torch_cuda_base

LABEL maintainer "User Name"


# Deal with getting tons of debconf messages
# See: https://github.com/phusion/baseimage-docker/issues/58
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections
# RUN apt-get update && apt-get install libusb-1.0-0 -y
# add GL:
RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config \
        libglvnd-dev \
        libgl1-mesa-dev \
        libegl1-mesa-dev \
        libgles2-mesa-dev && \
    rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Set timezone info
RUN apt-get update && apt-get install -y \
  tzdata \
  software-properties-common \
  && rm -rf /var/lib/apt/lists/* \
  && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
  && echo "America/Los_Angeles" > /etc/timezone \
  && dpkg-reconfigure -f noninteractive tzdata \
  && add-apt-repository -y ppa:git-core/ppa \
  && apt-get update && apt-get install -y \
  curl \
  lsb-core \
  wget \
  build-essential \
  cmake \
  git \
  git-lfs \
  iputils-ping \
  make \
  openssh-server \
  openssh-client \
  libeigen3-dev \
  libssl-dev \
  python3-pip \
  python3-ipdb \
  python3-tk \
  python3-wstool \
  sudo git bash unattended-upgrades \
  apt-utils \
  terminator \
  glmark2 \
  && rm -rf /var/lib/apt/lists/*

# push defaults to bashrc:
RUN apt-get update && apt-get install --reinstall -y \
  libmpich-dev \
  hwloc-nox libmpich12 mpich \
  && rm -rf /var/lib/apt/lists/*

# This is required to enable mpi lib access:
ENV PATH="${PATH}:/opt/hpcx/ompi/bin"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/hpcx/ompi/lib"



ENV TORCH_CUDA_ARCH_LIST "7.0+PTX"
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"

# Add cache date to avoid using cached layers older than this
ARG CACHE_DATE=2024-04-25


RUN pip install "robometrics[evaluator] @ git+https://github.com/fishbotics/robometrics.git"

# if you want to use a different version of curobo, create folder as docker/pkgs and put your
# version of curobo there. Then uncomment below line and comment the next line that clones from
# github

# COPY pkgs /pkgs

RUN mkdir /pkgs && cd /pkgs && git clone https://github.com/NVlabs/curobo.git

RUN cd /pkgs/curobo && pip3 install .[dev,usd] --no-build-isolation

WORKDIR /pkgs/curobo

# Optionally install nvblox:

# we require this environment variable to  render images in unit test curobo/tests/nvblox_test.py

ENV PYOPENGL_PLATFORM=egl

# add this file to enable EGL for rendering

RUN echo '{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}' >> /usr/share/glvnd/egl_vendor.d/10_nvidia.json

RUN apt-get update && \
    apt-get install -y libgoogle-glog-dev libgtest-dev curl libsqlite3-dev libbenchmark-dev && \
    cd /usr/src/googletest && cmake . && cmake --build . --target install && \
    rm -rf /var/lib/apt/lists/*

RUN cd /pkgs &&  git clone https://github.com/valtsblukis/nvblox.git && \
    cd nvblox && cd nvblox && mkdir build && cd build && \
    cmake .. -DPRE_CXX11_ABI_LINKABLE=ON && \
    make -j32 && \
    make install

RUN cd /pkgs && git clone https://github.com/nvlabs/nvblox_torch.git && \
    cd nvblox_torch && \
    sh install.sh $(python -c 'import torch.utils; print(torch.utils.cmake_prefix_path)') && \
    python3 -m pip install -e .

RUN python -m pip install pyrealsense2 opencv-python transforms3d

# install benchmarks:
RUN python -m pip install "robometrics[evaluator] @ git+https://github.com/fishbotics/robometrics.git"

# update ucx path: https://github.com/openucx/ucc/issues/476
RUN export LD_LIBRARY_PATH=/opt/hpcx/ucx/lib:$LD_LIBRARY_PATH


RUN wget 'https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-x64' -O /tmp/code_latest_amd64.deb && \
    sudo dpkg -i /tmp/code_latest_amd64.deb

# allias for code:
RUN echo "alias code='code --user-data-dir --no-sandbox'" >> /root/.bashrc

RUN wget 'http://archive.ubuntu.com/ubuntu/pool/main/libu/libusb-1.0/libusb-1.0-0_1.0.25-1ubuntu2_amd64.deb' -O /tmp/libusb-1.0-0_1.0.25-1ubuntu2_amd64.deb && \
    sudo dpkg -i /tmp/libusb-1.0-0_1.0.25-1ubuntu2_amd64.deb





##### Installing ROS Humble ######

# Définir des arguments pour désactiver les invites interactives pendant l'installation
ARG DEBIAN_FRONTEND=noninteractive

# Ajouter les dépôts ROS 2
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt-get update

# Installer des dépendances générales
RUN apt-get update && apt-get install -y \
    lsb-release \
    gnupg2 \
    curl \
    python3-pip \
    ros-humble-moveit \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Ajouter les sources de ROS 2 Humble
RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository universe
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Mettre à jour et installer ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-argcomplete \
    ros-humble-rviz2 \
    python3-colcon-common-extensions \
    ros-humble-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# COPY ros_packages/. /home/ros2_ws/src

# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /home/ros2_ws && colcon build"
# RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc


RUN sudo apt-get update && sudo apt-get install --reinstall -y \
    libmpich-dev \
    hwloc-nox libmpich12 mpich

# RUN mkdir src
WORKDIR /home/ros2_ws/src

RUN git clone https://github.com/Lab-CORO/CapacitiNet_msg
RUN cd .. && source /opt/ros/humble/setup.bash && colcon build

RUN git clone https://github.com/Lab-CORO/CapaciNet
RUN git clone https://github.com/Lab-CORO/curobo_ros
RUN cd .. && source /opt/ros/humble/setup.bash  && source ./install/setup.bash && colcon build


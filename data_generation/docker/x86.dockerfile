# add arg to specify the platform: rtx 4000 or rtx3000 (a100)
ARG PLATFORM=a100
FROM curobo_docker:${PLATFORM}
ARG GIT_USERNAME
ARG GIT_EMAIL

WORKDIR /home/ros2_ws/src
# RUN git clone https://github.com/Lab-CORO/CapaciNet.git  -b main

WORKDIR /home
RUN git clone https://github.com/rogersce/cnpy.git
RUN cd cnpy && mkdir build && cd build && cmake .. && make && make install

RUN git config --global user.name "$GIT_USERNAME" && \
    git config --global user.email "$GIT_EMAIL"

RUN sudo apt-get install libhdf5-openmpi-dev
RUN git clone --recursive https://github.com/BlueBrain/HighFive.git

WORKDIR /home/HighFive
RUN git checkout v2.8.0
RUN git submodule update --init --recursive

RUN cmake -DCMAKE_INSTALL_PREFIX=build/install -DHIGHFIVE_USE_BOOST=Off -B build .
RUN cmake --build build --parallel
RUN cmake --install build
RUN export CMAKE_PREFIX_PATH="/home/HighFive/build/install:${CMAKE_PREFIX_PATH}"

RUN apt update && apt install ccache -y
WORKDIR /home
RUN git clone https://github.com/isl-org/Open3D  
RUN cd Open3D && git checkout 1868f43 && util/install_deps_ubuntu.sh assume-yes

WORKDIR /home/Open3D
RUN mkdir build 
WORKDIR /home/Open3D/build 
RUN cmake .. -DBUILD_PYTHON_MODULE=OFF 
RUN make -j$(nproc) && make install

# Add curobo_ros packages needed
WORKDIR /home/ros2_ws
RUN cd src && git clone https://github.com/Lab-CORO/curobo_ros.git --recurse-submodules \
    && git clone https://github.com/Lab-CORO/CapaciNet.git && git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git

# Setup the workspace for curobo_ros and doosan
RUN apt install python3-vcstool && cd src/curobo_ros && vcs import < my.repos && \ 
     sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
     wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - &&\
     apt-get update && \
     apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-moveit-msgs\
        ros-humble-ros-gz-sim ros-humble-ros-gz libpoco-dev libyaml-cpp-dev\
        ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro\
        ros-humble-joint-state-publisher-gui ros-humble-ros2-control\
        ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs\
        dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group libignition-gazebo6-dev

ENV CMAKE_PREFIX_PATH='/home/HighFive/build/install:${CMAKE_PREFIX_PATH}'

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select curobo_msgs" && cd /home/ros2_ws && \
    . install/local_setup.bash

# Build workspace
# WORKDIR /home/ros2_ws
RUN /bin/bash -c "source /home/ros2_ws/install/setup.bash && \
    colcon build" && cd /home/ros2_ws && \
    . install/local_setup.bash

# RUN source /opt/ros/humble/setup.bash && \
#     cd /home/ros2_ws && \
#     . install/local_setup.bash


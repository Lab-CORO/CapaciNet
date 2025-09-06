FROM pytorch/pytorch

RUN conda install -c conda-forge pytorch-3dunet
RUN pip install tensorflow

# COPY ./ /workspace/capacitynet/ 


# Install ros2


RUN apt update &&  apt install curl -y
RUN apt install software-properties-common -y
RUN add-apt-repository universe




RUN apt update && apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ARG DEBIAN_FRONTEND=noninteractive
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \ 
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"  && \
    dpkg -i /tmp/ros2-apt-source.deb && \ 
    apt update && \
    apt install ros-humble-desktop ros-dev-tools ros-humble-nav2-msgs -y
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc


# Add curobo_msgs

RUN mkdir -p /workspace/capacitynet/ros2_ws/src && \
     cd /workspace/capacitynet/ros2_ws/src && git clone https://github.com/Lab-CORO/curobo_msgs 

RUN pip uninstall em && \
    pip install empy==3.3.4 && \
    cd /workspace/capacitynet/ros2_ws/src && \
    pip install lark catkin_pkg open3d


RUN  apt-get update && apt-get install -y ros-humble-rmw-cyclonedds-cpp ros-humble-cyclonedds

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENV PYTHONPATH=/opt/conda/lib/python3.10/site-packages:$PYTHONPATH
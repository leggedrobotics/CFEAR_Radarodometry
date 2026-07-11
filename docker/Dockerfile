# CFEAR Radar Odometry - ROS Noetic build for running on the Boreas dataset.
#
# Build (from the repository root, NOT from the docker/ folder):
#   docker build -f docker/Dockerfile -t cfear .
#
# See docker/README.md for how to convert Boreas data and run the odometry.

FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive

# --- System + build dependencies ---------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git \
        libceres-dev \
        libopencv-dev \
        libboost-all-dev \
        libeigen3-dev \
        python3-catkin-tools \
        python3-opencv \
        python3-numpy \
        ros-noetic-pcl-ros \
        ros-noetic-nav-msgs \
        ros-noetic-cv-bridge \
        ros-noetic-image-transport \
        ros-noetic-laser-geometry \
        ros-noetic-eigen-conversions \
        ros-noetic-tf-conversions \
    && rm -rf /var/lib/apt/lists/*

# --- Build the CFEAR catkin workspace ----------------------------------------
# The repository is copied in as the single package in the workspace. Its
# CMakeLists.txt does not actually require graph_map/ndt_map (they are only
# listed in package.xml), so it builds stand-alone.
WORKDIR /catkin_ws/src/cfear_radarodometry
COPY . .

WORKDIR /catkin_ws
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

# Source ROS + workspace automatically for every interactive/`docker run` shell.
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

WORKDIR /catkin_ws/src/cfear_radarodometry/docker
SHELL ["/bin/bash", "-c"]
CMD ["bash"]

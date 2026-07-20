# CFEAR Radar Odometry - online ROS2 Jazzy node.
#
# Build (from the repository root, NOT from ros2/docker/):
#   docker build -f ros2/docker/Dockerfile -t cfear_ros2 .
#
# Multi-arch: ros:jazzy-ros-base and every dependency below are published for
# amd64 AND arm64, and no x86-only compile flags are used, so the same file
# builds natively on an NVIDIA Jetson or cross-builds with
#   docker buildx build --platform linux/arm64 -f ros2/docker/Dockerfile -t cfear_ros2 .
# CFEAR is CPU-only (Ceres/OpenMP) - no CUDA/L4T base image needed.

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential git \
        libceres-dev \
        libpcl-dev \
        libopencv-dev \
        libboost-serialization-dev \
        libeigen3-dev \
        python3-colcon-common-extensions \
        ros-jazzy-pcl-conversions \
        ros-jazzy-cv-bridge \
        ros-jazzy-tf2-eigen \
        ros-jazzy-tf2-ros \
        ros-jazzy-angles \
        ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# The robot side runs CycloneDDS - use the same RMW to avoid cross-vendor
# discovery issues, and load the tuned config (bigger receive buffers for the
# ~1600 msg/s FFT stream).
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY ros2/docker/cyclonedds.xml /ws/config/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///ws/config/cyclonedds.xml

WORKDIR /ws
COPY ros2/navtech_msgs src/navtech_msgs
COPY ros2/cfear_radarodometry_ros2 src/cfear_radarodometry_ros2

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY ros2/tools /ws/tools

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /ws/install/setup.bash' >> /root/.bashrc

# Make `docker run cfear_ros2 ros2 launch ...` work without an interactive shell.
COPY ros2/docker/ros_entrypoint.sh /ros_entrypoint_cfear.sh
ENTRYPOINT ["/ros_entrypoint_cfear.sh"]
CMD ["bash"]

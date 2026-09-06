#!/bin/bash
# Run the online CFEAR odometry node (ROS2 Jazzy) against a live Navtech driver.
#
#   ./run_online.sh [extra ros2 launch args, e.g. image_topic:=/radar_data/radar_frame]
#
# Requirements on the host:
#   - image built:  docker build -f ros2/docker/Dockerfile -t cfear_ros2 .
#   - same ROS_DOMAIN_ID as the radar driver machine (default 0)
set -e

docker run --rm -it \
  --net=host --ipc=host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  cfear_ros2 \
  ros2 launch cfear_radarodometry_ros2 cfear_online.launch.py "$@"

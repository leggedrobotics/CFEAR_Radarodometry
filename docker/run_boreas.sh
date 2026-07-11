#!/bin/bash
# Run CFEAR Radar Odometry on one Boreas sequence (headless).
#
#   ./run_boreas.sh <boreas_sequence_dir> [output_dir]
#
# Steps:
#   1. Convert <seq>/radar/*.png -> <seq>/radar.bag  (skipped if bag exists)
#   2. Start a roscore (needed even offline: the node advertises topics)
#   3. Run offline_odometry with Boreas-tuned parameters
#
# Estimated trajectory is written to <output_dir>/est/01.txt (KITTI format).
set -e

SEQ_DIR="${1:?Usage: run_boreas.sh <boreas_sequence_dir> [output_dir]}"
SEQ_NAME="$(basename "${SEQ_DIR}")"
OUT_DIR="${2:-${SEQ_DIR}/cfear_output}"
BAG_PATH="${SEQ_DIR}/radar.bag"

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# --- 1. Convert to rosbag ----------------------------------------------------
# Include ground truth on /gt if applanix/radar_poses.csv is present.
GT_FLAG=""
if [ -f "${SEQ_DIR}/applanix/radar_poses.csv" ]; then
  GT_FLAG="--gt"
  echo "==> Found applanix/radar_poses.csv -> ground truth will be included."
else
  echo "==> No applanix/radar_poses.csv -> running odometry without ground truth."
fi

if [ ! -f "${BAG_PATH}" ]; then
  echo "==> Converting Boreas radar PNGs to rosbag..."
  python3 "$(dirname "$0")/boreas_to_rosbag.py" --seq "${SEQ_DIR}" --out "${BAG_PATH}" ${GT_FLAG}
else
  echo "==> Using existing bag: ${BAG_PATH}"
fi

# --- 2. roscore --------------------------------------------------------------
echo "==> Starting roscore..."
roscore >/dev/null 2>&1 &
ROSCORE_PID=$!
trap "kill ${ROSCORE_PID} 2>/dev/null || true" EXIT
until rostopic list >/dev/null 2>&1; do sleep 0.5; done

# --- 3. Odometry -------------------------------------------------------------
EST_DIR="${OUT_DIR}/est/"
GT_DIR="${OUT_DIR}/gt/"
mkdir -p "${EST_DIR}" "${GT_DIR}"

# CFEAR-3 parameters (best accuracy), adapted for Boreas.
#  - range_res 0.0596  : Navtech CIR204-H bin size (Oxford is 0.0438) -- REQUIRED
#  - dataset  oxford   : Boreas uses the Oxford polar image format
#  - radar_ccw true    : Boreas radar is mounted upright (Oxford is upside-down).
#                        If the trajectory comes out mirrored, flip this to false.
echo "==> Running CFEAR odometry (headless)..."
rosrun cfear_radarodometry offline_odometry \
  --bag_path        "${BAG_PATH}" \
  --sequence        "${SEQ_NAME}" \
  --est_directory   "${EST_DIR}" \
  --gt_directory    "${GT_DIR}" \
  --dataset         oxford \
  --range-res       0.0596 \
  --radar_ccw       false \
  --cost_type       P2P \
  --submap_scan_size 4 \
  --registered_min_keyframe_dist 1.5 \
  --res             3 \
  --k_strongest     40 \
  --z-min           60 \
  --weight_option   4 \
  --weight_intensity true \
  --loss_type       Huber \
  --loss_limit      0.1 \
  --disable_compensate false \
  --job_nr          1 \
  --method          cfear-3-boreas

echo "==> Done. Estimated trajectory: ${EST_DIR}"

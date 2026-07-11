#!/usr/bin/env python3
"""Convert a Boreas sequence's raw Navtech polar radar scans (and, optionally,
the Applanix ground-truth poses) into a rosbag that CFEAR Radarodometry reads.

Boreas stores each radar scan as a polar PNG under
    <sequence>/radar/<timestamp_microseconds>.png
with layout: rows = 400 azimuths, columns = [11 metadata bytes | range bins].
This is the same Navtech / Oxford format CFEAR already understands, so we simply
strip the 11 metadata columns and publish each scan as a sensor_msgs/Image on
/Navtech/Polar.

Ground truth (optional): Boreas provides <sequence>/applanix/radar_poses.csv with
columns  t,x,y,z,vx,vy,vz,r,p,y,wz,wy,wx  where t is the UTC timestamp in
microseconds (== the radar PNG filename), (x,y,z) is the radar position in a
global ENU frame, and (r,p,y) are roll/pitch/yaw. We reconstruct T_enu_radar
exactly as pyboreas does and publish it as nav_msgs/Odometry on /gt with the SAME
timestamp as the matching radar scan, so CFEAR's evaluator can pair them up.
CFEAR flattens this SE(3) pose to SE(2) internally (drops z/roll/pitch).

Usage:
    # radar only
    python3 boreas_to_rosbag.py --seq /data/seq --out /data/seq/radar.bag
    # radar + ground truth (auto-detects applanix/radar_poses.csv)
    python3 boreas_to_rosbag.py --seq /data/seq --out /data/seq/radar.bag --gt
"""
import argparse
import os
import sys

import cv2
import numpy as np
import rosbag
import rospy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

# Boreas/Oxford polar PNGs carry 11 metadata columns per row before the range
# data (8-byte timestamp + 2-byte sweep encoder + 1-byte valid flag).
METADATA_COLS = 11


def stamp_from_micros(t_us):
    """Build an exact rospy.Time from an integer microsecond UTC timestamp.

    Using integer arithmetic (not /1e6) guarantees the radar-image stamp and the
    ground-truth stamp derived from the same integer are bit-identical, so
    CFEAR's 0.1 ms timestamp matching pairs them reliably.
    """
    t_us = int(t_us)
    secs = t_us // 1_000_000
    nsecs = (t_us % 1_000_000) * 1000
    return rospy.Time(secs, nsecs)


# --- Rotation helpers, matching pyboreas exactly -----------------------------
def _roll(r):
    return np.array([[1, 0, 0],
                     [0, np.cos(r), np.sin(r)],
                     [0, -np.sin(r), np.cos(r)]], dtype=np.float64)


def _pitch(p):
    return np.array([[np.cos(p), 0, -np.sin(p)],
                     [0, 1, 0],
                     [np.sin(p), 0, np.cos(p)]], dtype=np.float64)


def _yaw(y):
    return np.array([[np.cos(y), np.sin(y), 0],
                     [-np.sin(y), np.cos(y), 0],
                     [0, 0, 1]], dtype=np.float64)


def enu_sensor_rot(r, p, y):
    """C_enu_sensor = roll(r) @ pitch(p) @ yaw(y)  (pyboreas convention)."""
    return _roll(r) @ _pitch(p) @ _yaw(y)


def rot_to_quat(C):
    """3x3 rotation matrix -> (x, y, z, w) quaternion."""
    tr = C[0, 0] + C[1, 1] + C[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (C[2, 1] - C[1, 2]) / s
        y = (C[0, 2] - C[2, 0]) / s
        z = (C[1, 0] - C[0, 1]) / s
    elif C[0, 0] > C[1, 1] and C[0, 0] > C[2, 2]:
        s = np.sqrt(1.0 + C[0, 0] - C[1, 1] - C[2, 2]) * 2
        w = (C[2, 1] - C[1, 2]) / s
        x = 0.25 * s
        y = (C[0, 1] + C[1, 0]) / s
        z = (C[0, 2] + C[2, 0]) / s
    elif C[1, 1] > C[2, 2]:
        s = np.sqrt(1.0 + C[1, 1] - C[0, 0] - C[2, 2]) * 2
        w = (C[0, 2] - C[2, 0]) / s
        x = (C[0, 1] + C[1, 0]) / s
        y = 0.25 * s
        z = (C[1, 2] + C[2, 1]) / s
    else:
        s = np.sqrt(1.0 + C[2, 2] - C[0, 0] - C[1, 1]) * 2
        w = (C[1, 0] - C[0, 1]) / s
        x = (C[0, 2] + C[2, 0]) / s
        y = (C[1, 2] + C[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def make_image_msg(polar_u8, stamp, frame_id="navtech"):
    height, width = polar_u8.shape
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.encoding = "mono8"
    msg.is_bigendian = 0
    msg.step = width
    msg.data = polar_u8.tobytes()
    return msg


def make_gt_msg(row, stamp, frame_id="world"):
    # row: t,x,y,z,vx,vy,vz,r,p,y,...
    C = enu_sensor_rot(row[7], row[8], row[9])
    qx, qy, qz, qw = rot_to_quat(C)
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.pose.position.x = row[1]
    msg.pose.pose.position.y = row[2]
    msg.pose.pose.position.z = row[3]
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    return msg


def load_gt(csv_path):
    """Return {timestamp_us(int): row(list[float])} from a Boreas pose CSV."""
    poses = {}
    with open(csv_path) as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) < 10:
                continue
            try:
                row = [float(p) for p in parts]
            except ValueError:
                continue  # header line
            poses[int(row[0])] = row
    return poses


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seq", required=True,
                    help="Path to a Boreas sequence dir (must contain radar/)")
    ap.add_argument("--out", default=None,
                    help="Output bag path (default: <seq>/radar.bag)")
    ap.add_argument("--gt", action="store_true",
                    help="Also write /gt from applanix/radar_poses.csv")
    ap.add_argument("--gt-csv", default=None,
                    help="Override path to the pose CSV (implies --gt)")
    ap.add_argument("--keep-metadata", action="store_true",
                    help="Do NOT strip the 11 metadata columns (debug only)")
    args = ap.parse_args()

    radar_dir = os.path.join(args.seq, "radar")
    if not os.path.isdir(radar_dir):
        sys.exit("ERROR: no 'radar/' folder in {}".format(args.seq))

    out_path = args.out or os.path.join(args.seq, "radar.bag")
    pngs = sorted(f for f in os.listdir(radar_dir) if f.endswith(".png"))
    if not pngs:
        sys.exit("ERROR: no .png scans in {}".format(radar_dir))

    gt_csv = args.gt_csv
    if gt_csv is None and args.gt:
        gt_csv = os.path.join(args.seq, "applanix", "radar_poses.csv")
    gt = {}
    if gt_csv:
        if not os.path.isfile(gt_csv):
            sys.exit("ERROR: ground-truth CSV not found: {}".format(gt_csv))
        gt = load_gt(gt_csv)
        print("Loaded {} ground-truth poses from {}".format(len(gt), gt_csv))

    print("Found {} radar scans in {}".format(len(pngs), radar_dir))
    print("Writing bag -> {}".format(out_path))

    n_gt_written = 0
    polar = None
    with rosbag.Bag(out_path, "w") as bag:
        for i, fname in enumerate(pngs):
            t_us = int(os.path.splitext(fname)[0])
            stamp = stamp_from_micros(t_us)

            polar = cv2.imread(os.path.join(radar_dir, fname), cv2.IMREAD_GRAYSCALE)
            if polar is None:
                print("  skipping unreadable {}".format(fname))
                continue
            if not args.keep_metadata:
                polar = polar[:, METADATA_COLS:]
            polar = np.ascontiguousarray(polar)
            bag.write("/Navtech/Polar", make_image_msg(polar, stamp), t=stamp)

            if t_us in gt:
                bag.write("/gt", make_gt_msg(gt[t_us], stamp), t=stamp)
                n_gt_written += 1

            if (i + 1) % 200 == 0 or i + 1 == len(pngs):
                print("  {}/{}".format(i + 1, len(pngs)))

    print("Done. Scan size (azimuths x range bins): {} x {}".format(
        polar.shape[0], polar.shape[1]))
    if gt:
        print("Wrote {}/{} radar scans with matching /gt poses".format(
            n_gt_written, len(pngs)))
        if n_gt_written == 0:
            print("WARNING: no GT timestamps matched radar filenames -- check the CSV.")


if __name__ == "__main__":
    main()

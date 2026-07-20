#!/usr/bin/env python3
"""Replay a Boreas sequence as live per-azimuth Navtech FFT messages (ROS2).

Reads <sequence>/radar/<timestamp_us>.png (rows = 400 azimuths, cols =
[11 metadata bytes | range bins]) and publishes:
  - one navtech_msgs/RadarConfigurationMsg (transient_local, like the driver)
  - navtech_msgs/RadarFftDataMsg per azimuth row on the FFT topic

This exercises the exact online pipeline of cfear_odometry_node without radar
hardware. The encoder tick is read from metadata bytes 8-9 (little-endian
uint16, Oxford/Boreas format); pass --synthetic-encoder to derive it from the
row index instead if your PNGs lack valid encoder values.

Typical use (inside the cfear_ros2 container, sequence bind-mounted):
    python3 /ws/tools/boreas_fft_replay.py --seq /data/seq
    python3 /ws/tools/boreas_fft_replay.py --seq /data/seq --rate 0   # fast as possible
"""
import argparse
import os
import struct
import random
import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy)

from navtech_msgs.msg import RadarConfigurationMsg, RadarFftDataMsg

METADATA_COLS = 11
AZIMUTHS = 400


def stamp_from_ns(t_ns, msg_time):
    msg_time.sec = int(t_ns // 1_000_000_000)
    msg_time.nanosec = int(t_ns % 1_000_000_000)


class BoreasFftReplay(Node):
    def __init__(self, args):
        super().__init__("boreas_fft_replay")
        self.args = args

        fft_qos = QoSProfile(
            depth=1600,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
            if args.fft_reliability == "best_effort"
            else QoSReliabilityPolicy.RELIABLE,
        )
        config_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.fft_pub = self.create_publisher(RadarFftDataMsg, args.fft_topic, fft_qos)
        self.config_pub = self.create_publisher(
            RadarConfigurationMsg, args.config_topic, config_qos)

    def publish_config(self, range_in_bins):
        msg = RadarConfigurationMsg()
        msg.azimuth_samples = AZIMUTHS
        msg.encoder_size = self.args.encoder_size
        msg.azimuth_offset = 0
        msg.bin_size = self.args.range_res
        msg.range_in_bins = range_in_bins
        msg.expected_rotation_rate = int(round(self.args.scan_rate)) or 4
        msg.range_gain = 1.0
        msg.range_offset = 0.0
        self.config_pub.publish(msg)
        self.get_logger().info(
            f"Published RadarConfigurationMsg: {AZIMUTHS} az, encoder "
            f"{msg.encoder_size}, {range_in_bins} bins, {msg.bin_size:.4f} m/bin")

    def replay(self):
        args = self.args
        radar_dir = os.path.join(args.seq, "radar")
        pngs = sorted(f for f in os.listdir(radar_dir) if f.endswith(".png"))
        if not pngs:
            sys.exit(f"ERROR: no .png scans in {radar_dir}")
        self.get_logger().info(f"Replaying {len(pngs)} scans from {radar_dir}")

        first = cv2.imread(os.path.join(radar_dir, pngs[0]), cv2.IMREAD_GRAYSCALE)
        range_in_bins = first.shape[1] - METADATA_COLS
        self.publish_config(range_in_bins)
        time.sleep(1.0)  # let subscriptions match before streaming

        scan_period = 1.0 / args.scan_rate if args.scan_rate > 0 else 0.0
        az_period = scan_period / AZIMUTHS

        for scan_idx, fname in enumerate(pngs):
            t_wall0 = time.monotonic()
            t_scan_us = int(os.path.splitext(fname)[0])
            polar = cv2.imread(os.path.join(radar_dir, fname), cv2.IMREAD_GRAYSCALE)
            if polar is None or polar.shape[0] != AZIMUTHS:
                self.get_logger().warning(f"skipping unreadable/odd-shaped {fname}")
                continue

            for row in range(AZIMUTHS):
                if args.drop_prob > 0 and random.random() < args.drop_prob:
                    continue
                meta = polar[row, :METADATA_COLS].tobytes()
                if args.synthetic_encoder:
                    tick = row * args.encoder_size // AZIMUTHS
                    t_az_ns = t_scan_us * 1000 + int(row * az_period * 1e9)
                else:
                    # Oxford/Boreas metadata: bytes 0-7 UNIX timestamp [us] LE,
                    # bytes 8-9 encoder tick LE, byte 10 valid flag
                    t_az_us = struct.unpack("<Q", meta[0:8])[0]
                    tick = struct.unpack("<H", meta[8:10])[0]
                    t_az_ns = t_az_us * 1000

                msg = RadarFftDataMsg()
                stamp_from_ns(t_az_ns, msg.header.stamp)
                msg.header.frame_id = "radar_link"
                msg.angle = tick * 360.0 / args.encoder_size
                msg.azimuth = tick
                msg.sweep_counter = scan_idx % 65536
                msg.ntp_seconds = int(t_az_ns // 1_000_000_000)
                msg.ntp_split_seconds = int(t_az_ns % 1_000_000_000)
                data = polar[row, METADATA_COLS:]
                msg.data = data.tobytes()
                msg.data_length = int(data.shape[0]) & 0xFFFF
                self.fft_pub.publish(msg)

            if (scan_idx + 1) % 50 == 0 or scan_idx + 1 == len(pngs):
                self.get_logger().info(f"  {scan_idx + 1}/{len(pngs)}")

            if scan_period > 0:
                sleep = scan_period - (time.monotonic() - t_wall0)
                if sleep > 0:
                    time.sleep(sleep)

        self.get_logger().info("Replay finished.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seq", required=True, help="Boreas sequence dir (contains radar/)")
    ap.add_argument("--fft-topic", default="/radar_data/fft")
    ap.add_argument("--config-topic", default="/radar_data/configuration_data")
    ap.add_argument("--fft-reliability", default="best_effort",
                    choices=["best_effort", "reliable"])
    ap.add_argument("--scan-rate", type=float, default=4.0,
                    help="Scans per second; 0 = as fast as possible (default 4)")
    ap.add_argument("--encoder-size", type=int, default=5600,
                    help="Encoder ticks per rotation reported in the config msg")
    ap.add_argument("--range-res", type=float, default=0.0596)
    ap.add_argument("--synthetic-encoder", action="store_true",
                    help="Derive encoder ticks from the row index instead of PNG metadata")
    ap.add_argument("--drop-prob", type=float, default=0.0,
                    help="Randomly drop this fraction of azimuths (tests robustness)")
    args = ap.parse_args()

    rclpy.init()
    node = BoreasFftReplay(args)
    try:
        node.replay()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

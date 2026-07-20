# CFEAR Radar Odometry — online ROS2 Jazzy port

Online radar odometry from a live Navtech radar: subscribes the driver's
**per-azimuth FFT messages**, assembles full polar rotations, runs the
CFEAR-3 pipeline in-process, and publishes `nav_msgs/Odometry` on
**`/odometry`** (plus an optional `odom -> radar_link` TF).

Two wire formats are supported, selected by `radar.message_format`:
- `navtech_msgs` (default) — `navtech_msgs/RadarFftDataMsg` +
  `RadarConfigurationMsg`, the official Navtech IA SDK messages (plain-typed
  fields), default topic `/radar_data/fft`.
- `legacy_bytes` — `messages/RadarFftDataMessage` + `RadarConfigurationMessage`,
  the leggedrobotics `navtech_radar_ros` driver fork (every scalar field packed
  as a network-order `uint8[]` byte array), default topic `/radar_data/fft_data`.
  See [cfear_radarodometry_ros2/include/cfear_radarodometry/legacy_radar_codec.h](cfear_radarodometry_ros2/include/cfear_radarodometry/legacy_radar_codec.h)
  for the decode and its caveats (`bin_size` in particular), and
  [cfear_radarodometry_ros2/config/cfear3_b2w_ras3.yaml](cfear_radarodometry_ros2/config/cfear3_b2w_ras3.yaml)
  for the b2w_rsl / RAS-3 preset. The two formats are NOT wire compatible with
  each other.

The ROS1 tree at the repository root is untouched — the offline Boreas
pipeline in [../docker/](../docker/) keeps working as before.

```
ros2/
├── navtech_msgs/             vendored Navtech IA SDK messages (navtech_msgs format)
├── messages/                 vendored navtech_radar_ros-fork messages (legacy_bytes format)
├── cfear_radarodometry_ros2/ the port: assembler + filter + registration + node
├── docker/                   Dockerfile (Jazzy), CycloneDDS config, run script
└── tools/boreas_fft_replay.py  hardware-free end-to-end test from Boreas PNGs (navtech_msgs format)
```

## Build & run (Docker)

From the **repository root**:

```bash
docker build -f ros2/docker/Dockerfile -t cfear_ros2 .

# host kernel serves the sockets under --net=host -> raise the receive buffer once:
sudo sysctl -w net.core.rmem_max=10485760

./ros2/docker/run_online.sh                       # defaults: /radar_data/fft -> /odometry
./ros2/docker/run_online.sh fft_topic:=/my/fft odom_topic:=/my/odom
```

The image sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (the robot runs
CycloneDDS; mixing DDS vendors is a classic silent-failure source) and loads
[docker/cyclonedds.xml](docker/cyclonedds.xml) with a 10 MB receive buffer for
the ~1600 msg/s FFT stream. Make sure `ROS_DOMAIN_ID` matches the driver
machine; `--net=host` (set by `run_online.sh`) is required for DDS discovery.

**Jetson / arm64:** the same Dockerfile builds natively on a Jetson (or
cross-build with `docker buildx build --platform linux/arm64 ...`). CFEAR is
CPU-only, so no CUDA base image or NVIDIA runtime is needed.

## Radar configuration

Geometry (azimuth count, encoder size, range bins, bin size) is taken from the
driver's `RadarConfigurationMsg` on `/radar_data/configuration_data`. If none
arrives within `radar.config_timeout_sec` (5 s), the node falls back to the
`radar.*` parameters in
[cfear_radarodometry_ros2/config/cfear3_boreas.yaml](cfear_radarodometry_ros2/config/cfear3_boreas.yaml)
— defaults match a Boreas-like CIR204-H (400 azimuths, encoder 5600,
0.0596 m/bin, 4 Hz). For a different sensor (e.g. a future RAS3) adjust those
parameters or just rely on the configuration message.

Algorithm parameters are the CFEAR-3 set validated offline on Boreas
(`cost_type P2P, submap 4, keyframe 1.5 m, res 3, k=40, z_min 60, Huber 0.1`).

## b2w_rsl deployment (NavTech RAS-3, legacy_bytes)

Run with `params_file:=$(ros2 pkg prefix --share cfear_radarodometry_ros2)/config/cfear3_b2w_ras3.yaml`
against the leggedrobotics `navtech_radar_ros` fork's `/radar_data/fft_data` +
`/radar_data/configuration_data` topics. This preset sets
`radar.message_format: legacy_bytes` and publishes odometry on
`/cfear/odometry` (not `/odometry` — `fognav_replay` also publishes there from
dataset ground truth; never run both against the same topic). See the yaml's
header comment for the geometry assumptions that still need hardware
verification (`encoder_size`, `rotation_rate_hz`, `legacy_bin_size_scale`).

## Testing without hardware (Boreas replay)

Terminal 1 — odometry node; Terminal 2 — replay a Boreas sequence as FFT
messages; Terminal 3 — record:

```bash
docker run --rm -it --net=host --ipc=host -v /path/to/boreas-seq:/data/seq cfear_ros2
# T1:
ros2 launch cfear_radarodometry_ros2 cfear_online.launch.py
# T2 (same container via `docker exec`, or a second `docker run`):
python3 /ws/tools/boreas_fft_replay.py --seq /data/seq            # real time, 4 Hz
python3 /ws/tools/boreas_fft_replay.py --seq /data/seq --drop-prob 0.01  # robustness test
# T3:
ros2 topic hz /radar_data/fft          # ~1600 Hz
ros2 topic echo /odometry --field pose.pose
ros2 bag record -o cfear_run /odometry
```

Evaluate against ground truth / the ROS1 result with evo:

```bash
pip3 install --user evo
evo_traj bag2 cfear_run /odometry --save_as_tum
# GT: reuse the applanix pose math from ../docker/boreas_to_rosbag.py to make a TUM file
evo_ape tum gt.tum odometry.tum -a --plot_mode xy --save_plot ape.pdf
```

Sanity anchor: the ROS1 offline pipeline (`../docker/run_boreas.sh`) on the
same sequence runs the identical algorithm and parameters — the two
trajectories should agree closely. A large deviation means a port bug (check
row order, `fuser.radar_ccw`, `radar.range_res`).

**Validation status (boreas-2021-09-02-11-42, first ~1425 scans / 2 km):**

- `offline_png_test` (ported library fed PNGs directly): matches the ROS1
  reference trajectory to 7.3 m after 2.3 km (≈0.3%, Ceres-version noise).
  Rerun with `ros2 run cfear_radarodometry_ros2 offline_png_test
  /data/seq/radar out.tum 1500 1`.
- Full online path (replay → assembler → node): 1.5% final difference vs ROS1
  with exact encoder ticks, 2.1% with the real (jittery) Boreas encoder
  metadata, zero registration failures, real-time at 2× speed (8 Hz).
- `fuser.radar_ccw: true` is the validated value for Boreas — the ROS1
  reference run (`cfear_output/pars.txt`: "radar reversed, true") used it too.

## Troubleshooting a silent topic

1. `ros2 topic info /radar_data/fft -v` — the node subscribes BEST_EFFORT by
   default; a RELIABLE-only publisher still matches, but if the driver
   publishes with incompatible QoS set `radar.fft_reliability`.
2. Config never arrives: some drivers publish it volatile-only — set
   `radar.config_durability: volatile` (you then need the driver started
   *after* this node, or rely on the parameter fallback).
3. Driver forks publishing `RadarFftDataMessage` with byte-array fields (e.g.
   leggedrobotics' `navtech_radar_ros`) are **not wire compatible** with the
   vendored `navtech_msgs` (official SDK, plain-typed fields) — set
   `radar.message_format: legacy_bytes` (see above) instead of writing a
   translation node.
4. `radar_ccw`: if the live trajectory mirrors reality, flip `fuser.radar_ccw`.

## Notes vs the ROS1 code

- rviz marker visualization (`MapPointNormal::PublishMap` etc.) is stubbed out;
  the registered point clouds are still published (`out.publish_registered_cloud`).
- Registration failure no longer terminates the process: the node logs an error
  and falls back to the constant-velocity prediction for that scan.
- If registration is slower than the scan rate, the newest scan replaces the
  queued one (bounded latency); drops are logged.

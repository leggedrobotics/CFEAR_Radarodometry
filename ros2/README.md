# CFEAR Radar Odometry — online ROS2 Jazzy port

Online radar odometry from a live Navtech radar: subscribes the driver's
**whole-rotation polar frames**, runs the CFEAR-3 pipeline in-process, and
publishes `nav_msgs/Odometry` on **`/odometry`** (plus an optional
`odom -> radar_link` TF).

Input is `sensor_msgs/Image` polar frames (mono8, rows=azimuths, leading 11
Oxford/Boreas metadata columns per row) from the leggedrobotics
`navtech_radar_sdk` `polar_image_publisher` or `radarsplat_replay`'s frames
mode, default topic `/radar_data/radar_frame`, with a latched
`navtech_msgs/RadarConfigurationMsg` on `/radar_data/configuration_data`
announcing the layout. See
[cfear_radarodometry_ros2/config/cfear3_b2w_ras3.yaml](cfear_radarodometry_ros2/config/cfear3_b2w_ras3.yaml)
for the b2w_rsl / RAS-3 preset.

The node never sees per-azimuth FFT spokes: the publisher hands over rotations
already assembled, so there is no spoke assembler here.

The ROS1 tree at the repository root is untouched — the offline Boreas
pipeline in [../docker/](../docker/) keeps working as before.

```
ros2/
├── navtech_msgs/             vendored Navtech IA SDK messages (RadarConfigurationMsg)
├── cfear_radarodometry_ros2/ the port: filter + registration + node
└── docker/                   Dockerfile (Jazzy), CycloneDDS config, run script
```

## Build & run (Docker)

From the **repository root**:

```bash
docker build -f ros2/docker/Dockerfile -t cfear_ros2 .

# host kernel serves the sockets under --net=host -> raise the receive buffer once:
sudo sysctl -w net.core.rmem_max=10485760

./ros2/docker/run_online.sh                       # defaults: /radar_data/radar_frame -> /odometry
./ros2/docker/run_online.sh image_topic:=/my/frames odom_topic:=/my/odom
```

The image sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (the robot runs
CycloneDDS; mixing DDS vendors is a classic silent-failure source) and loads
[docker/cyclonedds.xml](docker/cyclonedds.xml) with a 10 MB receive buffer for
the whole-rotation frames. Make sure `ROS_DOMAIN_ID` matches the driver
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

## b2w_rsl deployment (NavTech RAS-3, polar_image)

Run with `params_file:=$(ros2 pkg prefix --share cfear_radarodometry_ros2)/config/cfear3_b2w_ras3.yaml`
against the leggedrobotics `navtech_radar_sdk` driver's
`/radar_data/radar_frame` + `/radar_data/configuration_data` topics. This
preset publishes odometry on `/cfear/odometry` (not `/odometry` —
`fognav_replay` also publishes there from dataset ground truth; never run both
against the same topic). Against dataset replay, prefer
`radar.stamp_source: header` (replayed metadata carries dataset-era time).

### Which instant the odometry is stamped with (`radar.stamp_source`)

The published pose refers to **mid-sweep** and always has: the fuser deskews
every scan with `Compensate()`, whose `GetRelTimeStamp()` =
`±(atan2(y,x)/2π − 0.5)` is zero at half a rotation, so every point is
transported to the middle of the acquisition interval. Stamp it with anything
else and the pose is mis-dated by up to a full sweep — 250 ms at 4 Hz.

Two families of values, and the difference between them is not cosmetic:

| family | values | meaning |
| --- | --- | --- |
| **time-indexed** | `sweep_mid`, `sweep_start`, `sweep_end` | min / mid / max over the valid rows' embedded times. Independent of the publisher's tick→row mapping, and invariant under `radar.reverse_azimuths`. |
| **row-indexed** | `first_row`, `last_row` (aliases `first_azimuth`, `last_azimuth`) | the embedded time found at the lowest / highest valid **row index**. Says nothing about acquisition order on its own. |
| neither | `header`, `now` | publisher's header stamp / wall clock. |

`first_azimuth` and `last_azimuth` are **deprecated** and the node warns when
they are used. They are honest on Boreas, whose rows ascend in time, and a trap
on the b2w RAS-3: `polar_image_publisher`'s
`row = round(tick*400/16000) % 400` wraps tick 16000 onto row 0, so **row 0
holds the sweep's last spoke**. There `first_azimuth` (row 0) and
`last_azimuth` (row 399) are the same instant to within 0.6 ms — both at the
sweep **end**, ~125 ms after the pose they label.

For the same reason, mid-sweep must **not** be derived as
`(stamp_first_row + stamp_last_row)/2`: on the RAS-3 both operands sit at the
sweep end, so their midpoint is the sweep end and is wrong by the full 125 ms.
`stamp_sweep_mid()` uses the min and max of the valid rows' times instead.

Measured on `20260825_231849_hg_nav_1` (`/radar_data/radar_frame`, 400/400 rows
valid): `header.stamp` matches the earliest spoke (row 1) to within 1 µs, row 0
carries `header + 249.1 ms`, the sweep spans 249.1 ms, and `sweep_mid` lands at
`header + 124.6 ms`. Replaying that bag through the node confirms the published
`/cfear/odometry` stamps match the computed midpoint to 0.2 µs over 154
messages.

`cfear3_b2w_ras3.yaml` uses `sweep_mid`. `cfear3_boreas.yaml` stays on
`first_row` so the validated Boreas runs keep reproducing — those stamps are the
sweep start, i.e. ~125 ms *early*; move it to `sweep_mid` when there is an
appetite to re-baseline.

## Testing without hardware (Boreas replay)

Terminal 1 — odometry node; Terminal 2 — replay a Boreas sequence as polar
frames with `radarsplat_replay` (`publish_mode: frames`, in the RadarSplat
repo's replay container); Terminal 3 — record:

```bash
docker run --rm -it --net=host --ipc=host -v /path/to/boreas-seq:/data/seq cfear_ros2
# T1:
ros2 launch cfear_radarodometry_ros2 cfear_online.launch.py
# T2 (replay container): ros2 launch radarsplat_replay dataset_replay.launch.py \
#     data_dir:=/path/to/dataset
# T3:
ros2 topic hz /radar_data/radar_frame  # ~4 Hz
ros2 topic echo /odometry --field pose.pose
ros2 bag record -o cfear_run /odometry
```

No replay container at hand? `offline_png_test` runs the same library straight
off the sequence PNGs, no DDS involved (see the validation note below).

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
- Full online path (replay → node): 1.5% final difference vs ROS1 with exact
  encoder ticks, 2.1% with the real (jittery) Boreas encoder metadata, zero
  registration failures, real-time at 2× speed (8 Hz). Measured through the
  since-removed spoke path; the frame path feeds the same assembled scans in.
- `fuser.radar_ccw: true` is the validated value for Boreas — the ROS1
  reference run (`cfear_output/pars.txt`: "radar reversed, true") used it too.

## Troubleshooting a silent topic

1. `ros2 topic info /radar_data/radar_frame -v` — the node subscribes RELIABLE,
   matching the driver and `radarsplat_replay`'s frame publishers; a BEST_EFFORT
   publisher will not match it.
2. Config never arrives: some drivers publish it volatile-only — set
   `radar.config_durability: volatile` (you then need the driver started
   *after* this node, or rely on the parameter fallback).
3. Frame width mismatches the announced `[start_bin, end_bin)` — the node warns
   and uses the frame as published; check the driver's range crop, or cap the
   consumed bins with `radar.max_range_bins`.
4. `radar_ccw`: if the live trajectory mirrors reality, flip `fuser.radar_ccw`.

## Notes vs the ROS1 code

- rviz marker visualization (`MapPointNormal::PublishMap` etc.) is stubbed out;
  the registered point clouds are still published (`out.publish_registered_cloud`).
- Registration failure no longer terminates the process: the node logs an error
  and falls back to the constant-velocity prediction for that scan.
- If registration is slower than the scan rate, the newest scan replaces the
  queued one (bounded latency); drops are logged.

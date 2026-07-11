# Running CFEAR Radar Odometry on Boreas (Docker, headless)

CFEAR reads a **rosbag** with radar scans on the `/Navtech/Polar` topic
(`sensor_msgs/Image`). Boreas ships raw Navtech polar PNGs instead, so the flow is:

```
Boreas <seq>/radar/*.png  --boreas_to_rosbag.py-->  radar.bag  --offline_odometry-->  est/01.txt
```

Boreas uses the same Navtech CIR204-H / Oxford polar format, so the existing
`dataset=oxford` code path works. The one parameter that **must** change is the
range resolution: **0.0596 m** for Boreas (Oxford is 0.0438).

## 1. Build the image

From the **repository root** (one level above this `docker/` folder):

```bash
docker build -f docker/Dockerfile -t cfear .
```

## 2. Run on a sequence

Mount the folder that contains your Boreas sequence and run the helper. A Boreas
sequence directory looks like `boreas-2020-11-26-13-58/` and must contain a
`radar/` subfolder of `<timestamp>.png` scans.

```bash
docker run --rm -it \
  -v /path/to/boreas-2020-11-26-13-58:/data/seq \
  cfear \
  ./run_boreas.sh /data/seq
```

`run_boreas.sh` will:
1. Convert `/data/seq/radar/*.png` → `/data/seq/radar.bag` (skipped if it exists).
2. Start a `roscore` (required even offline — the node advertises topics).
3. Run `offline_odometry` with CFEAR-3 parameters tuned for Boreas.

If `applanix/radar_poses.csv` is present in the sequence, ground truth is
automatically included on `/gt` and a matching `gt/` trajectory is written.

> **Note:** step 1 is skipped if `radar.bag` already exists. If you ran the
> odometry once *without* ground truth, or you change `--radar_ccw` /
> `TIMESTAMP_UNITS_PER_SECOND`, delete the stale bag first so it gets rebuilt:
> `rm "$BOREAS_SEQ/radar.bag"`.

Because the sequence dir is bind-mounted, outputs land on your host at:

```
/path/to/boreas-2020-11-26-13-58/
├── radar.bag                 # generated rosbag (/Navtech/Polar [+ /gt])
└── cfear_output/
    ├── est/
    │   ├── 01.txt            # estimated trajectory, KITTI 3x4 pose-per-line
    │   ├── tum_01.txt        # same trajectory, TUM (t x y z qx qy qz qw)
    │   └── cov_01.txt        # per-pose covariance
    └── gt/                   # only if radar_poses.csv was present
        ├── 01.txt            # ground-truth trajectory, KITTI format
        └── tum_01.txt        # ground-truth trajectory, TUM format
```

## 3. Things to verify on your first run

These depend on sensor mounting/conventions and are worth a quick sanity check:

- **Scan dimensions** printed by the converter should be `400 x ~3768`
  (azimuths × range bins). If width is ~3779 you left the 11 metadata columns in.
- **Trajectory shape / direction**: plot `tum_01.txt` (e.g. with `evo_traj tum`).
  If the path is mirrored or turns the wrong way, flip `--radar_ccw` in
  [run_boreas.sh](run_boreas.sh) (Boreas radar is upright; set to `false` if wrong).
- **Timestamp unit**: the converter assumes Boreas filenames are microseconds.
  If motion compensation looks off, change `TIMESTAMP_UNITS_PER_SECOND` in
  [boreas_to_rosbag.py](boreas_to_rosbag.py).

## Comparing 2D odometry against the 3D Applanix ground truth

CFEAR estimates only planar motion (SE(2): x, y, yaw); Boreas ground truth is
full SE(3). You do not compare in 3D — you reduce the GT to the same plane:

- `boreas_to_rosbag.py` reconstructs `T_enu_radar` (radar pose in a global ENU
  frame) from `radar_poses.csv` using the exact pyboreas convention
  (`C = roll(r)·pitch(p)·yaw(y)`).
- CFEAR's evaluator flattens that SE(3) pose to SE(2), dropping z / roll / pitch,
  and expresses both est and GT relative to their first pose — so both end up in
  the *radar-frame-at-t₀* and are directly comparable.
- GT and radar-scan messages share identical integer-µs timestamps, so the
  evaluator's timestamp matching pairs them exactly.

### Where each trajectory starts (important)

Both `est/` and `gt/` are independently anchored to **identity at the first
frame**, so both files start at the origin (0, 0). CFEAR does **no** est↔gt
registration — `One2OneCorrespondance` only time-interpolates GT onto the est
timestamps. So if a plot shows est starting *away* from the origin while gt sits
at (0, 0), that offset was introduced by **evo's `-a` (Umeyama) alignment**, which
fits one rigid transform over the whole trajectory to minimise total error and
therefore does not pin the start poses together. That is expected, not a bug.

### evo (quick plots + ATE/RPE)

```bash
pip3 install --user evo   # once

# (A) Raw comparison — both already start at the origin, so do NOT align:
evo_traj tum cfear_output/est/tum_01.txt \
  --ref cfear_output/gt/tum_01.txt --plot_mode xy -p

# (B) Pin only the first pose together (rigid) — nicest visual overlay:
evo_traj tum cfear_output/est/tum_01.txt \
  --ref cfear_output/gt/tum_01.txt --align_origin --plot_mode xy -p

# (C) ATE / RPE numbers — use -a (Umeyama), the convention for reporting ATE:
evo_ape tum cfear_output/gt/tum_01.txt cfear_output/est/tum_01.txt -a --plot_mode xy -p
evo_rpe tum cfear_output/gt/tum_01.txt cfear_output/est/tum_01.txt -a --plot_mode xy -p
```

Use (A)/(B) for trajectory figures and (C) for the ATE metric. Over SSH without a
display, replace `-p` with `--save_plot out.pdf`.

`--align_origin` only removes a rigid offset — it will **not** hide a mirrored
trajectory, so it stays a valid check for the `--radar_ccw` flag: if est and GT
come out mirror-imaged, flip `--radar_ccw` in [run_boreas.sh](run_boreas.sh),
delete `radar.bag`, and re-run.

### KITTI drift (the metric used in the CFEAR paper)

Translation error % and rotation deg/m over segments, invariant to any global
frame rotation:

```bash
git clone https://github.com/dan11003/radar_kitti_benchmark
# point its evaluation at cfear_output/est and cfear_output/gt (01.txt files)
```

## Parameter presets

[run_boreas.sh](run_boreas.sh) uses **CFEAR-3** (best accuracy). To try the
lighter CFEAR-2 / CFEAR-1 presets, see the parameter blocks in
[../launch/oxford_demo](../launch/oxford_demo) — only swap the algorithm params;
keep `--range-res 0.0596` and `--dataset oxford`.

## Later: RViz visualization

This setup is headless. To watch the live map, run the container with X11
forwarding (`--net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`) and
`roslaunch cfear_radarodometry vis.launch` alongside the odometry.

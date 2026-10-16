# Testing on real sensors

Handoff guide for moving the ARCO people detection & tracking stack from the
Gazebo simulator to a real robot with real sensors. The code was migrated to
ROS 2 Humble and verified in simulation; this document covers what changes for
real hardware. See [README.md](README.md) for the full architecture.

## TL;DR — what's different from simulation

The stack is fully parameterized, so going to real sensors is **configuration,
not code**. Three things change:

| Sim | Real robot |
|---|---|
| `use_sim_time:=true` | **drop it** (defaults to `false`; real sensors use wall-clock) |
| `scan_topic:=/scan_raw`, camera topics remapped to sim names | remap to your **real driver topics** |
| `frame_id:=odom` (sim `map` was unreliable) | `frame_id:=map` is normally fine with real localization (AMCL/SLAM) |

## Status: what's verified vs. open

- ✅ **Laser detector + fuser**: verified end-to-end in sim (tracks walking
  people correctly against ground truth).
- ⚠️ **Camera detector**: both `people_rgbd` (HOG) and `people_yolo` (YOLOv4-tiny)
  run correctly but did **not** reliably detect the simulator's *synthetic*
  human meshes. On a **real camera** they are expected to work much better —
  **this is the main thing to validate on real hardware.**
- ⚠️ **UWB branch** (`localino`/`roloc`): functional, validated with synthetic
  range data only — never tested against real UWB anchors/tags.

## Prerequisites

```bash
# ROS 2 Humble + dependencies
sudo apt install libgsl-dev libopencv-dev \
     ros-humble-cv-bridge ros-humble-image-transport \
     ros-humble-message-filters ros-humble-tf2-ros \
     ros-humble-tf2-geometry-msgs ros-humble-vision-opencv

# Build
cd <ros2_ws>
colcon build --packages-up-to \
  people_laser_detector people_laser_tracker \
  people_rgbd people_yolo range_msgs localino roloc arco_people_tracker
source install/setup.bash
```

## Running with real sensors

First identify your real topics and frames:
```bash
ros2 topic list                 # find your scan + camera topics
ros2 run tf2_tools view_frames  # confirm the TF tree / frame names
```

### 1. Laser detector
```bash
ros2 launch people_laser_tracker people_tracker.launch.py \
  scan_topic:=/scan \
  frame_id:=map \
  params_file:=/path/to/laser_tracker_params.yaml
```

### 2. Camera detector (YOLO — recommended over HOG)
Remap each image topic to your driver's names. Example for a RealSense:
```bash
ros2 launch people_yolo people_yolo.launch.py \
  rgb_image_topic:=/camera/color/image_raw \
  depth_image_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=map \
  params_file:=/path/to/people_yolo_params.yaml
```
- The node handles both `32FC1` (meters) and `16UC1` (millimeters) depth.
- **Camera frame:** real drivers publish a proper optical frame
  (e.g. `camera_color_optical_frame`), so you can usually leave
  `camera_optical_frame` empty in the config — the node uses the image's
  header frame. (In sim we had to inject a static optical frame because the
  sim stamped images with `base_footprint`.)
- Depth and RGB **must be registered/aligned** (same resolution & viewpoint).

### 3. UWB branch (optional)
```bash
ros2 launch localino localino.launch.py params_file:=/path/to/anchors.yaml
ros2 launch roloc   roloc.launch.py   params_file:=/path/to/tags.yaml
```
Put the **real surveyed anchor positions** in `localino`'s `anchor_list` and the
**real tag IDs** in `roloc`'s per-node sections.

### 4. Fuser
```bash
ros2 launch arco_people_tracker people_tracker.launch.py \
  frame_id:=map \
  params_file:=/path/to/people_tracker_params.yaml
```
`frame_id` must match the frame the detectors publish in.

## How to verify it's working

1. **Detectors publishing** — each enabled detector should publish on
   `/people_detections` with its tag:
   ```bash
   ros2 topic echo /people_detections | grep -A1 tagnames   # LASER / RGBD / UWB
   ```
2. **Fused output** — consolidated tracks on `/people`:
   ```bash
   ros2 topic echo /people
   ```
3. **Visual (RViz)** — Fixed Frame = your `frame_id`; add MarkerArray displays
   on `/people_detections/markers` and `/people/markers`. Markers should sit on
   the real people and follow them.
4. **Sanity** — a walking person should show a nonzero, realistic velocity
   (~0.3–1.5 m/s) and a stable position; static false positives sit at ~0 m/s.

## Things to watch for

- **TF tree must connect** the sensor frames to `frame_id`. If you see
  `TF ... failed` warnings, the transform from the scan/camera frame to
  `frame_id` isn't available — check localization and `ros2 run tf2_ros tf2_echo`.
- **Laser false positives**: 2D leg detection latches onto chair/table legs.
  Tune `min_beams` / `threshold` / `max_distance`; the camera branch helps
  disambiguate.
- **Laser coverage**: a 2D laser misses occluded or standing-still people —
  expected; this is why the system is multi-sensor.
- **Camera performance**: YOLOv4-tiny runs ~28 fps on CPU. If too slow, lower
  `input_size` (320) or `rate`; for better accuracy use a larger YOLO model or
  a GPU backend. **This is where the TFG student's "update the detector" work
  fits** — swapping in a newer model that publishes to `/people_detections`
  with the `RGBD` tag requires no changes to the fuser.

## Key parameters (in `config/*.yaml`)

- Laser: `threshold`, `min_beams`, `max_distance`, `legs_distance`,
  `track_distance`, `track_timeout`
- YOLO: `confidence_threshold`, `nms_threshold`, `input_size`, `depth_min/max`,
  `rate`, `camera_optical_frame`
- UWB: `anchor_list` (localino), `tag` / `name` per node (roloc)
- Fuser: `track_distance`, `track_timeout`

Repo: https://github.com/ppueyor/people_tracking_ros2

# people_tracking_ros2

Multi-sensor people detection and tracking for **ROS 2 Humble**, ported from the
original ROS 1 stack. Three independent detectors (2D LiDAR, RGB-D camera, UWB
tags) each publish person detections to a shared topic, and a fuser consolidates
them into robust, Kalman-filtered tracks.

```
 SENSORS              DETECTORS                        FUSER                 OUTPUT
─────────         ──────────────────              ──────────────          ──────────
 LiDAR    ──►  people_laser_tracker ──┐
 Camera   ──►  people_rgbd / people_yolo ─┼──► /people_detections ──► arco_people_tracker ──► /people
 UWB tags ──►  localino → roloc      ──┘     (people_msgs/People,         (assoc + KF)        (FUSED
                                              tagged LASER/RGBD/UWB)                            tracks)
```

Every detector speaks the same message (`people_msgs/People`) on the same topic
(`/people_detections`), tagging each detection with its source
(`LASER` / `RGBD` / `UWB`). The fuser associates detections to tracks, updates a
Kalman filter per person (weighting by per-sensor noise), and republishes merged
tracks on `/people` tagged `FUSED`.

## Packages

| Package | Lang | Role |
|---|---|---|
| `people_laser/people_laser_detector` | C++ | Scan segmentation + AdaBoost leg classifier (library) |
| `people_laser/people_laser_tracker` | C++ | LiDAR leg-pair detector + per-person Kalman tracker → `LASER` |
| `people_rgbd` | C++ | RGB-D HOG+SVM person detector → `RGBD` |
| `people_yolo` | Python | RGB-D YOLOv4-tiny (OpenCV DNN) person detector → `RGBD` (modern alternative to `people_rgbd`) |
| `people_uwb/range_msgs` | — | Custom UWB range messages (`P2PRange`, `P2PRangeWithPose`) |
| `people_uwb/localino` | C++ | UWB anchor-gateway UDP driver |
| `people_uwb/roloc` | C++ | Range-only EKF localization of a tagged person → `UWB` |
| `arco_people_tracker` | C++ | Multi-sensor fuser → `/people` tagged `FUSED` |

## Dependencies

- **ROS 2 Humble** (Ubuntu 22.04)
- `people_msgs` (from the ROS `people` stack)
- System libraries:
  ```bash
  sudo apt install libgsl-dev libopencv-dev \
       ros-humble-cv-bridge ros-humble-image-transport \
       ros-humble-message-filters ros-humble-tf2-ros \
       ros-humble-tf2-geometry-msgs ros-humble-vision-opencv
  ```
  - `libgsl-dev` is required by `people_laser_detector`.
  - `people_yolo` uses the OpenCV DNN module already bundled with `python3-opencv` — **no torch/CUDA needed**.

## Build

```bash
cd <your_ros2_ws>
colcon build --packages-up-to \
  people_laser_detector people_laser_tracker \
  people_rgbd people_yolo \
  range_msgs localino roloc \
  arco_people_tracker
source install/setup.bash
```

## Quick start (Gazebo simulator)

> The included `kill_all.sh` cleans up Gazebo/ROS processes — run it first if a
> previous session left orphan processes (the usual cause of a sim that won't
> relaunch).

```bash
./kill_all.sh
```

**1. Simulator** (provides `/scan_raw`, camera topics, and walking people):
```bash
ros2 launch hunav_gazebo_wrapper pmb2_cafe.launch.py
```

**2. Laser detector** (the reliable path):
```bash
ros2 launch people_laser_tracker people_tracker.launch.py \
  use_sim_time:=true scan_topic:=/scan_raw frame_id:=odom
```

**3. Fuser:**
```bash
ros2 launch arco_people_tracker people_tracker.launch.py \
  use_sim_time:=true frame_id:=odom people_topic:=/people_tracker/people
```

**4. View the result:**
```bash
ros2 topic echo /people_tracker/people      # fused tracks (tagged FUSED)
```
In RViz: Fixed Frame `odom`; add MarkerArray displays on
`/people_detections/markers` (green = detections) and `/people/markers`
(white = fused tracks).

### Simulator gotchas

- **Always pass `use_sim_time:=true`** to every node, or track-timeout logic
  compares wall-clock time against sim-time stamps and drops every track.
- **Use `frame_id:=odom`, not `map`** — this sim's AMCL does not keep `map→odom`
  continuously published, so `map`-frame transforms are unreliable.
- The camera is stamped `base_footprint` with no optical frame; the YOLO/RGBD
  branch therefore needs a static optical-frame TF (see below).

## Running on a real robot

The nodes are parameterized for deployment — nothing sim-specific is hard-coded.
Each node takes a `params_file` and topic-remap launch arguments; edit the
shipped `config/*.yaml` (or pass your own) and point the topics at your driver:

```bash
# Laser
ros2 launch people_laser_tracker people_tracker.launch.py \
  scan_topic:=/your/scan params_file:=/path/to/laser.yaml

# Camera (YOLO) — remap each image topic to your driver's names
ros2 launch people_yolo people_yolo.launch.py \
  rgb_image_topic:=/camera/color/image_raw \
  depth_image_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  params_file:=/path/to/yolo.yaml

# UWB
ros2 launch localino localino.launch.py params_file:=/path/to/anchors.yaml
ros2 launch roloc roloc.launch.py params_file:=/path/to/tags.yaml

# Fuser
ros2 launch arco_people_tracker people_tracker.launch.py params_file:=/path/to/fuser.yaml
```

Key config values: `frame_id` (output frame, default `map`), `track_distance`,
`track_timeout`, per-detector thresholds, the UWB `anchor_list` (real surveyed
anchor positions), and the per-person `tag` IDs for `roloc`.

If your camera driver stamps images with a non-optical frame, set
`camera_optical_frame` in the YOLO/RGBD config to your camera's optical frame
(real drivers like realsense2_camera publish one, e.g.
`camera_color_optical_frame`).

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/people_detections` | `people_msgs/People` | detectors → fuser (tagged LASER/RGBD/UWB) |
| `/people_detections/markers` | `visualization_msgs/MarkerArray` | detector viz |
| `/people` | `people_msgs/People` | fuser output (tagged FUSED) |
| `/people/markers` | `visualization_msgs/MarkerArray` | fuser viz |

## Status / known issues

- **Laser detector + fuser**: verified working end-to-end.
- **`people_yolo`**: runs on CPU (~28 fps) but detection of the *simulator's*
  synthetic human meshes is unreliable — the model is trained on real imagery
  and should perform far better on a real camera. HOG (`people_rgbd`) has the
  same limitation, more so.
- The UWB branch is functional but was validated with synthetic range data, not
  real hardware.

## License

TODO

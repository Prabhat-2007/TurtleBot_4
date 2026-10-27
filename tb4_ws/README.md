# tb4_ws — standalone TurtleBot 4 (standard) for Gazebo Harmonic

Only the TurtleBot 4 **URDF + meshes** are kept from the stock stack. Everything else
(`turtlebot4_simulator`, `irobot_create_*`, `ros2_control`, Create 3 nodes) is replaced by three small packages:

| Package | What it does |
|---|---|
| `tb4_description` | Self-contained xacro/URDF + meshes: Create 3 base, shell, tower, RPLIDAR A1, OAK-D Pro **stereo pair** (left/right cameras, 7.5 cm baseline). Gazebo sensors and the diff-drive plugin are defined in the URDF. |
| `tb4_bringup` | `sim.launch.py`: Gazebo + spawn + `ros_gz_bridge` + robot_state_publisher + stereo depth + RViz, one command. Worlds: `empty`, `arena` (your arena, sensors enabled). |
| `tb4_drive` | `keyboard_teleop`, `velocity_gate` (limits, accel ramp, timeout, lidar safety stop), `sensor_demo` (how to read lidar + stereo in your own code). |

Drive is Gazebo's native `DiffDrive` plugin, so there is no `controller_manager`, no `diff_drive_controller`,
and no iRobot Create 3 stack. Bumper, cliff, IR, wheel-drop, mouse and button simulation were dropped.

## Requirements
Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic:
```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
     ros-jazzy-joint-state-publisher ros-jazzy-stereo-image-proc ros-jazzy-rviz2
```

## Build
```bash
cd ~/tb4_ws && rosdep install --from-path src -yi && colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 launch tb4_bringup sim.launch.py                       # empty world, RViz, stereo depth
ros2 launch tb4_bringup sim.launch.py world:=arena x:=1.0 y:=0.5 yaw:=1.57
ros2 launch tb4_bringup sim.launch.py namespace:=runner      # topics under /runner/...
ros2 launch tb4_bringup sim.launch.py gui:=false rviz:=false stereo_proc:=false   # light / headless

ros2 run tb4_drive keyboard_teleop          # second terminal
ros2 run tb4_drive sensor_demo              # prints lidar + camera stats
ros2 launch tb4_description display.launch.py   # view the model only, no sim
```
Launch args: `world namespace x y z yaw gui rviz drive stereo_proc lidar_samples lidar_rate stereo_width stereo_height stereo_fps`.
On a laptop with a discrete NVIDIA GPU, export `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` first.

## Topics (relative to the namespace)
| Topic | Type | Notes |
|---|---|---|
| `cmd_vel_raw` | Twist | **Send commands here** (teleop, your script). Goes through `velocity_gate`. |
| `cmd_vel` | Twist | Gate output -> Gazebo. Publish here directly only to bypass limits/safety (or run with `drive:=false`). |
| `odom`, `tf`, `joint_states`, `imu` | | odom -> base_link TF comes from Gazebo |
| `scan` | LaserScan | RPLIDAR, frame `rplidar_link` (mounted yaw +90° as on the real robot), 720 pts, 10 Hz, 0.164–12 m |
| `stereo/left/image_raw`, `stereo/right/image_raw` | Image (rgb8) | 640x400 @ 30 Hz, 80° HFOV, frames `oakd_{left,right}_camera_optical_frame` |
| `stereo/left/camera_info`, `stereo/right/camera_info` | CameraInfo | right camera carries Tx = -fx·baseline |
| `stereo/disparity`, `stereo/points2` | DisparityImage, PointCloud2 | from `stereo_image_proc` (when `stereo_proc:=true`) |

Nav2 users: remap its output to `cmd_vel_raw`, or set `drive:=false` and let it publish `cmd_vel`.
Safety-gate tuning is in `tb4_drive/velocity_gate.py` (ROS parameters: `stop_distance`, `slow_distance`, `safety_enabled`, ...).

## Status / caveats
Written and statically checked (xacro expands for both empty and named namespaces, TF tree is a single tree
rooted at `base_link`, safety logic unit-tested). It was **not run in Gazebo** in the authoring environment.
Things to look at first if something is off: camera/lidar `frame_id`s in RViz (set via `<gz_frame_id>`),
`stereo/points2` being empty (check `camera_info/p[3]` is non-zero on the right camera), and the DiffDrive
plugin tags in `tb4_description/urdf/gazebo.xacro`.

Licenses: meshes/URDF derived from `turtlebot4` (Apache-2.0) and `create3_sim` (BSD).

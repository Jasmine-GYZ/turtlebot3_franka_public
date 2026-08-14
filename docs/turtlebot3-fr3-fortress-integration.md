# TurtleBot3 + FR3 Fortress Integration — Complete

## Overview

Replaced OpenMANIPULATOR-X (4 DOF) with FR3 arm (7 DOF + 2-finger hand) on TurtleBot3 Waffle Pi in Gazebo Fortress (Ignition Gazebo). All 5 controllers work, all 3 sensors work, cmd_vel works, robot moves. Ready for navigation and object recognition.

## Repositories Cloned

- `src/franka_ros2` (humble branch) — NOT compiled, only used as reference
- `src/franka_description` (tag 2.8.1) — compiled, provides FR3 URDF/meshes via `$(find franka_description)`

## Files Modified (5 files)

### 1. `src/turtlebot3_simulations/turtlebot3_manipulation_gazebo/urdf/turtlebot3_manipulation.urdf.xacro`
- Replaced OpenMANIPULATOR-X includes with FR3 includes (`franka_arm.xacro`, `franka_hand.xacro`, utils)
- Replaced `base_fixed` joint (base_link → link1) with `fr3_base_mount` (base_link → fr3_link0, same offset xyz="-0.092 0.0 0.091")
- Replaced `<xacro:open_manipulator_x>` + `<xacro:open_manipulator_x_gazebo>` with `<xacro:franka_arm>` (connected_to="", gazebo=true) + `<xacro:franka_hand>` (connected_to="fr3_link8", ee_id="franka_hand_white")
- Key: franka_arm is called with connected_to="" so we manually handle attachment; franka_hand reads `inertials.yaml` from franka_description

### 2. `src/turtlebot3_simulations/turtlebot3_manipulation_gazebo/ros2_control/turtlebot3_manipulation_system.ros2_control.xacro`
- Replaced `joint1~4` + `gripper_left_joint` + `gripper_right_joint` with:
  - `fr3_joint1~7`: position command, limits from franka_description joint_limits.yaml, folded transport pose initial values
  - `fr3_finger_joint1`: position command, limits 0.0~0.04, initial 0.04 (open)
- `fr3_finger_joint2` NOT in ros2_control — stays as URDF mimic handled by Gazebo physics
- Wheel joints and IMU sensor unchanged

### 3. `src/turtlebot3_simulations/turtlebot3_manipulation_gazebo/config/hardware_controller_manager.yaml`
- arm_controller joints: `joint1~4` → `fr3_joint1~7`
- gripper_controller joint: `gripper_left_joint` → `fr3_finger_joint1`

### 4. `src/turtlebot3_simulations/turtlebot3_manipulation_gazebo/urdf/turtlebot3_waffle_pi.urdf.xacro`
- camera_joint z: `0.084` → `0.80` (camera height from ground ≈ 0.81m, above table height ~0.76m)

### 5. `src/turtlebot3_simulations/turtlebot3_manipulation_gazebo/package.xml`
- Added `<exec_depend>franka_description</exec_depend>`

## NOT Modified (intentionally left as-is)
- `config/gazebo_controller_manager.yaml` — referenced by Gazebo plugin but spawner overrides params via hardware_controller_manager.yaml; outdated joint list doesn't affect functionality
- `launch/turtlebot3_franka.launch.py` — not changed; same controllers, same bridges, same spawner sequence

## Architecture Design Decisions

1. **Single ros2_control plugin**: Only one `gz_ros2_control::GazeboSimROS2ControlPlugin` for entire robot (TB3 + FR3). FR3's `franka_arm.gazebo.xacro` was NOT used because it creates its own world link + ros2_control block.

2. **Position control, not effort**: FR3 arm uses position command interface (same as OpenMANIPULATOR-X). FR3 demo's effort-based approach needs `GazeboGravityCompensationSystem` which is incompatible with shared `GazeboSimSystem`. Position mode is sufficient for navigation phase.

3. **URDF mimic for finger_joint2**: ros2_control doesn't reliably support mimic/multiplier. Only `fr3_finger_joint1` in ros2_control; `fr3_finger_joint2` stays as URDF mimic (Gazebo physics enforces it).

4. **FR3 arm link prefix**: `arm_prefix=""` and `connected_to=""` on franka_arm macro. Links named `fr3_link0` through `fr3_link8`, joints named `fr3_joint1` through `fr3_joint7`. Attachment handled manually via `fr3_base_mount` fixed joint.

## Transport Pose (folded arm for navigation)

| Joint | Initial Value (rad) |
|---|---|
| fr3_joint1 | 0.0 |
| fr3_joint2 | -pi/4 ≈ -0.785 |
| fr3_joint3 | 0.0 |
| fr3_joint4 | -3*pi/4 ≈ -2.356 |
| fr3_joint5 | 0.0 |
| fr3_joint6 | pi/2 ≈ 1.571 |
| fr3_joint7 | pi/4 ≈ 0.785 |
| fr3_finger_joint1 | 0.04 (open) |

## Launch & Test

```bash
# Build
cd /home/tina/turtlebot3_ws
colcon build --symlink-install --packages-select franka_description turtlebot3_manipulation_gazebo

# Launch
source install/setup.bash
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py

# Verify
ros2 control list_controllers  # all 5 should be active
ign topic -l | grep -E "scan|pi_camera"  # sensors
ros2 topic hz /scan  # ~4.7 Hz
ros2 topic hz /camera/image_raw  # ~15-20 Hz
```

## Verified Test Results (2026-08-10)
- 5/5 controllers: Configured and activated ✅
- LiDAR /scan: 4.7 Hz ✅
- RGB camera: 18 Hz ✅
- Depth camera: 15 Hz ✅
- cmd_vel → robot moves (tested: 0.2 m/s for 4s → odom x=0.808m) ✅
- FR3 joints all at correct transport pose values ✅

## Next Steps (not done yet)
- navigation ( nav2)
- Object recognition (YOLO or similar) using elevated camera
- Tasks: navigate to rooms, count objects, mark positions on /map
- Later: arm manipulation (may need effort control upgrade)

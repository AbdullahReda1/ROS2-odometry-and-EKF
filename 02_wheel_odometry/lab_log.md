# Session 2 Lab Log: Wheel Odometry & `diff_drive_controller`

**Workspace:** `~/ROS2-odometry-and-EKF/02_wheel_odometry`
**Robot / Controller:** `diff_drive_robot` (`myrobot`) / `diff_drive_controller` (ROS 2 Jazzy)

---

## 1. Controllers & Hardware Interfaces Verification

```bash
# Verify active controllers in controller_manager
ros2 control list_controllers
# Output:
# diff_drive_controller   diff_drive_controller/DiffDriveController      active
# joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster  active

# Verify command and state interfaces exposed by gz_ros2_control
ros2 control list_hardware_interfaces
# Command interfaces:
#   left_wheel_joint/velocity  [available] [claimed]  <-- Claimed by diff_drive_controller
#   right_wheel_joint/velocity [available] [claimed]  <-- Claimed by diff_drive_controller
# State interfaces:
#   left_wheel_joint/position, left_wheel_joint/velocity
#   right_wheel_joint/position, right_wheel_joint/velocity
```

---

## 2. Kinematics, Feedback & Frame Parameters

```bash
# Confirm joint names match URDF exactly
ros2 param get /diff_drive_controller left_wheel_names    # ['left_wheel_joint']
ros2 param get /diff_drive_controller right_wheel_names   # ['right_wheel_joint']

# Confirm physical dimensions match URDF (cylinder radius = 0.05, y offsets = +0.12 / -0.12 -> sep = 0.24)
ros2 param get /diff_drive_controller wheel_radius        # Double value is: 0.05
ros2 param get /diff_drive_controller wheel_separation    # Double value is: 0.24

# Confirm TF broadcasting and encoder feedback mode
ros2 param get /diff_drive_controller enable_odom_tf      # Boolean value is: True (will disable when EKF takes over)
ros2 param get /diff_drive_controller position_feedback   # Boolean value is: True (integrates wheel positions, not velocities)
```

---

## 3. Topics, Frame Headers & Publish Rate

```bash
# Active ROS 2 graph topics (-t shows message types)
ros2 topic list -t
# Key topics:
#   /clock [rosgraph_msgs/msg/Clock]
#   /diff_drive_controller/cmd_vel [geometry_msgs/msg/TwistStamped]
#   /diff_drive_controller/odom [nav_msgs/msg/Odometry]
#   /joint_states [sensor_msgs/msg/JointState]
#   /tf [tf2_msgs/msg/TFMessage]
#   /tf_static [tf2_msgs/msg/TFMessage]

# Verify odometry frame headers
ros2 topic echo /diff_drive_controller/odom --once --field header.frame_id   # odom
ros2 topic echo /diff_drive_controller/odom --once --field child_frame_id    # base_link

# Check live publish frequency (configured publish_rate: 50.0 Hz)
ros2 topic hz /diff_drive_controller/odom
# Measured average rate: ~48.8 - 49.2 Hz (min: 0.019s, max: 0.026s)
```

---

## 4. Live Wheel Joint States & TF2 Transform (`odom -> base_link`)

### Wheel Joint Feedback (`/joint_states`)

```bash
ros2 topic echo /joint_states --once
# header.frame_id: base_link
# name: ['left_wheel_joint', 'right_wheel_joint']
# position: [8.8013 rad, 11.2015 rad]   <-- Right wheel turned further -> net left (CCW) turn
# velocity: [~0.0 rad/s, ~0.0 rad/s]    <-- Robot stationary at snapshot
```

### Live Transform Tracking (`tf2_echo`)

```bash
ros2 run tf2_ros tf2_echo odom base_link
# Note: Initial "Invalid frame ID 'odom'" warning is normal until first TF message arrives.
```

| Sim Time (s)             | Translation `[x, y, z]` (m) | Yaw Orientation (rad / deg)  | Movement Observation                                 |
| :----------------------- | :-------------------------- | :--------------------------- | :--------------------------------------------------- |
| `589.53` – `590.49`      | `[0.250, 0.000, 0.000]`     | `0.000 rad` (`0.000°`)       | Robot drove straight `0.25 m` along +X and paused.   |
| `591.49`                 | `[0.484, 0.058, 0.000]`     | `0.487 rad` (`27.892°`)      | Robot curved forward and turned left (CCW).          |
| `592.41` – `596.25`      | `[0.490, 0.061, 0.000]`     | `0.500 rad` (`28.650°`)      | Robot stopped at final pose (`~28.65°` left).        |

---

## 5. Active Controller Parameter Dump (`ros2 param dump`)

```bash
ros2 param dump /diff_drive_controller
```

### Key Active Settings Summary

* **Timing & Execution:** `update_rate: 100`, `publish_rate: 50.0`, `use_sim_time: true`, `is_async: false`
* **Odometry Mode:** `position_feedback: true`, `open_loop: false`, `velocity_rolling_window_size: 10`
* **Calibration Multipliers:** `wheel_separation_multiplier: 1.0`, `left_wheel_radius_multiplier: 1.0`, `right_wheel_radius_multiplier: 1.0`
* **Covariance Arrays:** `pose_covariance_diagonal` and `twist_covariance_diagonal` are still `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` (must be set non-zero before EKF fusion).
* **YAML Note:** Unset optional limits appear as `.nan` in `ros2 param dump`; omit `.nan` lines when copying into `controllers.yaml`.

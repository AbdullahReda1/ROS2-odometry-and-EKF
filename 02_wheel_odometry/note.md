# Session 2: Wheel Odometry with `diff_drive_controller`

> **Goal:** Understand how `diff_drive_controller` turns wheel joint feedback into `/diff_drive_controller/odom` and the `odom -> base_link` TF transform, and which parameters control odometry quality.

---

## 1. Core Architecture & Hardware Abstraction

`diff_drive_controller` is a `ros2_control` plugin that performs two jobs every control cycle (`100 Hz` in your `controller_manager`):

1. **Drive (Command Path):** Converts incoming velocity commands (`/diff_drive_controller/cmd_vel`, `geometry_msgs/msg/TwistStamped`) into individual left and right wheel angular velocities.
2. **Odometry (Feedback Path):** Reads wheel joint feedback, computes the dead-reckoning step, and publishes `nav_msgs/msg/Odometry` plus `/tf` (`odom -> base_link`).

### Why `ros2_control` Uses Hardware Interfaces

`ros2_control` never talks directly to Gazebo or physical motors—it reads and writes to **named interfaces** (`position`, `velocity`). Because the controller only interacts with these abstract interfaces, **your controller code and `controllers.yaml` remain identical** when moving from simulation to real hardware; you only swap the `<hardware><plugin>` tag in your URDF.

| Environment | `<hardware><plugin>` Tag | What Fills the Interfaces |
| :--- | :--- | :--- |
| **Gazebo Sim (Jazzy / Harmonic)** | `gz_ros2_control/GazeboSimSystem` | Gazebo physics engine (your current setup). |
| **Gazebo Classic** | `gazebo_ros2_control/GazeboSystem` | Legacy Gazebo simulator. |
| **Mock / Math-Only Testing** | `mock_components/GenericSystem` | Echoes commands directly to state without physics. |
| **Real Robot (Existing Driver)** | e.g. `diffdrive_arduino/DiffDriveArduinoHardware` | Microcontroller / motor driver over Serial or CAN. |
| **Real Robot (Custom C++)** | `my_robot_hardware/MyMotorDriver` | Custom class inheriting `hardware_interface::SystemInterface`. |

---

## 2. Controller Inputs, Outputs & Step Computation

### What `diff_drive_controller` Reads and Writes

| Direction | Interface / Topic | Type | Notes |
| :--- | :--- | :--- | :--- |
| **Reads** | `left_wheel_joint/position`, `right_wheel_joint/position` | State Interface | Used when `position_feedback: true` (default). |
| **Reads** | `left_wheel_joint/velocity`, `right_wheel_joint/velocity` | State Interface | Used only if `position_feedback: false`. |
| **Reads** | `/diff_drive_controller/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Target linear (`x`) and angular (`z`) velocity. |
| **Writes** | `left_wheel_joint/velocity`, `right_wheel_joint/velocity` | Command Interface | Target wheel speeds sent to hardware. |
| **Writes** | `/diff_drive_controller/odom` | `nav_msgs/msg/Odometry` | Estimated pose, twist, and `6x6` covariances. |
| **Writes** | `odom -> base_link` | `/tf` | Broadcast only when `enable_odom_tf: true`. |

### Why Position Feedback Beats Velocity Feedback

* **Position feedback (`position_feedback: true`):** Wheel position is a cumulative running total. Even if a control cycle jitters or misses a sample, the next sample still captures the full distance traveled.
* **Velocity feedback (`position_feedback: false`):** Must be multiplied by $\Delta t$ at every step, so timing jitter or a delayed sample permanently introduces distance error.

### How Each Step Is Computed

1. **Simulation Time ($\Delta t$):** Uses elapsed time from `/clock` (`use_sim_time: true`). Even if Gazebo runs at `0.5x` wall-clock speed, $\Delta t$ stays synchronized with physics.
2. **Exact vs. RK2 Integration:**
   * **Straight motion ($\Delta\theta \approx 0$):** Uses an exact straight-line step.
   * **Turning motion ($\Delta\theta \neq 0$):** Uses **2nd-Order Runge-Kutta (RK2)**—evaluating the heading at the midpoint ($\theta + \Delta\theta / 2$) so curved paths do not drift outward.

---

## 3. Critical Parameters & Your Verified Robot Configuration

### URDF vs. `controllers.yaml` Consistency Check

`diff_drive_controller` reads dimensions from `controllers.yaml`, **not** from `robot.urdf`. Both files must match:

| Parameter | `controllers.yaml` Value | `robot.urdf` Verification | Impact if Mismatched |
| :--- | :--- | :--- | :--- |
| `wheel_radius` | `0.05` m | `<cylinder radius="0.05" .../>` | Linear distance scales incorrectly. |
| `wheel_separation` | `0.24` m | Left `y = +0.12`, Right `y = -0.12` ($0.12 - (-0.12) = 0.24$ m) | Turn angles are consistently over- or under-estimated. |
| `left_wheel_names` | `["left_wheel_joint"]` | `<joint name="left_wheel_joint" ...>` | Controller fails to read/command left wheel. |
| `right_wheel_names` | `["right_wheel_joint"]` | `<joint name="right_wheel_joint" ...>` | Controller fails to read/command right wheel. |
| `odom_frame_id` | `odom` | Matches RViz `Fixed Frame: odom` | Frame mismatch breaks TF lookup. |
| `base_frame_id` | `base_link` | Root link `<link name="base_link">` | Robot model detaches from odometry frame. |

### Verified Live Parameter Dump (`ros2 param dump /diff_drive_controller`)

From your running node inspection:

* `position_feedback: true` and `open_loop: false` (confirmed reading actual encoder positions).
* `publish_rate: 50.0` Hz (note: `ros2 topic hz` reports ~25 Hz in wall time because Gazebo runs at ~0.5x real time).
* Calibration multipliers are at neutral defaults (`wheel_separation_multiplier: 1.0`, `left_wheel_radius_multiplier: 1.0`, `right_wheel_radius_multiplier: 1.0`).
* **Important YAML rule:** Do **not** copy `.nan` entries (`max_jerk: .nan`, `max_deceleration: .nan`, etc.) from `ros2 param dump` into `controllers.yaml`—omit them so the parser does not fail.
* **Jazzy note:** `use_stamped_vel` was removed in ROS 2 Jazzy (`TwistStamped` is always used).

### Clean `controllers.yaml` Template (With Useful Non-NaN Defaults)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    use_sim_time: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]

    wheel_separation: 0.24
    wheel_radius: 0.05
    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0

    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_link
    enable_odom_tf: true
    tf_frame_prefix_enable: true
    tf_frame_prefix: ""

    position_feedback: true
    open_loop: false
    velocity_rolling_window_size: 10
    publish_limited_velocity: false
    cmd_vel_timeout: 0.5

    # Currently 0.0; populate with non-zero variances before enabling EKF (Session 5/6)
    pose_covariance_diagonal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    twist_covariance_diagonal: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    linear.x.has_velocity_limits: true
    linear.x.max_velocity: 0.5
    linear.x.min_velocity: -0.5
    linear.x.has_acceleration_limits: true
    linear.x.max_acceleration: 1.0

    angular.z.has_velocity_limits: true
    angular.z.max_velocity: 2.0
    angular.z.min_velocity: -2.0
    angular.z.has_acceleration_limits: true
    angular.z.max_acceleration: 2.0
```

---

## 4. The Single-Publisher Rule (`odom -> base_link`)

* **Rule:** Exactly **one** node may broadcast `odom -> base_link` on `/tf`.
* **Right now (Session 2):** Keep `enable_odom_tf: true` in `diff_drive_controller` because no EKF is running yet.
* **Later (Session 6):** When `ekf_node` (`robot_localization`) takes over publishing `odom -> base_link`, you **must** set `enable_odom_tf: false` in `controllers.yaml`. Leaving both enabled causes two conflicting transforms, making the robot flicker and jump in RViz2.

---

## 5. Practical & Troubleshooting Commands

### Inspect Hardware Interfaces & Active Controllers

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

*Verify `left_wheel_joint` and `right_wheel_joint` show `[available] [claimed]` under command interfaces and `[available]` under `position` and `velocity` state interfaces.*

### Inspect & Export Controller Parameters

```bash
ros2 param list /diff_drive_controller
ros2 param get /diff_drive_controller wheel_radius
ros2 param get /diff_drive_controller wheel_separation
ros2 param get /diff_drive_controller position_feedback
ros2 param dump /diff_drive_controller
```

### Debug "Robot Moves in Gazebo, but `/odom` Stays at Zero"

```bash
# 1. Confirm wheel encoder positions are actually changing
ros2 topic echo /joint_states --once

# 2. Confirm controller joint names and frames match URDF
ros2 param get /diff_drive_controller left_wheel_names
ros2 param get /diff_drive_controller odom_frame_id

# 3. Monitor live transform and publish rate
ros2 topic hz /diff_drive_controller/odom
ros2 run tf2_ros tf2_echo odom base_link
```

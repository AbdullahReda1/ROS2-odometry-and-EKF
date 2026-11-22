# ROS 2 Odometry & Gazebo Lab Log (Annotated Reference)

---

## 1. Topics & Active Controllers

Verify that the differential drive controller is active and inspect the exact topic names and message types.

```bash
# List odometry and velocity command topics
ros2 topic list | grep -E "odom|cmd_vel"
# Output:
# /diff_drive_controller/cmd_vel
# /diff_drive_controller/odom

# Verify controller states
ros2 control list_controllers
# Output:
# diff_drive_controller   diff_drive_controller/DiffDriveController      active
# joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster  active

# Check message types and publisher/subscriber counts
ros2 topic info /diff_drive_controller/cmd_vel   # Type: geometry_msgs/msg/TwistStamped (Pub: 1, Sub: 1)
ros2 topic info /diff_drive_controller/odom      # Type: nav_msgs/msg/Odometry (Pub: 1, Sub: 0)
```

---

## 2. Message Definitions (`Odometry` & `TwistStamped`)

Inspecting the structure of incoming velocity commands and outgoing odometry estimates.

```bash
# View full message layouts
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show geometry_msgs/msg/TwistStamped
```

* **`nav_msgs/msg/Odometry` Key Fields:**
  * `header.frame_id`: `"odom"` (parent fixed frame relative to spawn).
  * `child_frame_id`: `"base_link"` (robot body frame).
  * `pose.pose`: 3D position `(x, y, z)` and quaternion orientation `(x, y, z, w)`.
  * `pose.covariance`: 36-element array (flattened `6x6` uncertainty matrix).
  * `twist.twist`: Linear `(x, y, z)` and angular `(x, y, z)` velocities in `child_frame_id`.
  * `twist.covariance`: 36-element velocity uncertainty matrix.
* **`geometry_msgs/msg/TwistStamped` Key Fields:**
  * Required by ROS 2 Jazzy `diff_drive_controller` by default (includes `header` stamp + `twist` velocities).

---

## 3. Commanding Motion & Inspecting Odometry Fields

Publish a velocity command and extract specific subfields from `/diff_drive_controller/odom`.

```bash
# Drive forward at 0.2 m/s (published at 1 Hz for 30 iterations)
ros2 topic pub -r 1 -t 30 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: auto, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"

# Read full single message (note: "A message was lost!!!" is normal CLI QoS drop noise)
ros2 topic echo /diff_drive_controller/odom --once

# Extract specific nested fields cleanly using dot notation
ros2 topic echo /diff_drive_controller/odom --field header.frame_id --once        # odom
ros2 topic echo /diff_drive_controller/odom --field child_frame_id --once         # base_link
ros2 topic echo /diff_drive_controller/odom --field header.stamp.sec --once       # Simulation time (sec)
ros2 topic echo /diff_drive_controller/odom --field pose.pose.position --once     # {x: 1.0428, y: -0.2170, z: 0.0}
ros2 topic echo /diff_drive_controller/odom --field pose.pose.orientation.z --once # 0.9997
ros2 topic echo /diff_drive_controller/odom --field twist.twist.linear --once     # ~0.0 when stopped
```

> **Tip:** Using `--field <path>` avoids printing the 36-element zero arrays every time you check position or timestamps.

---

## 4. Covariance Parameters & Diagnostics

Check whether `diff_drive_controller` is publishing valid uncertainty values for downstream EKF fusion.

```bash
# Find covariance parameters on the controller node
ros2 param list | grep -i covariance
# Output: pose_covariance_diagonal, twist_covariance_diagonal

# Read current values (currently unconfigured / all zeros)
ros2 param get /diff_drive_controller pose_covariance_diagonal
# Double values are: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

ros2 param get /diff_drive_controller twist_covariance_diagonal
# Double values are: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# View official parameter descriptions & recommended defaults
ros2 param describe /diff_drive_controller pose_covariance_diagonal
ros2 param describe /diff_drive_controller twist_covariance_diagonal
```

* **Diagnostic Finding:** Both parameters are currently `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`, resulting in all-zero `36`-element covariance arrays on `/diff_drive_controller/odom`.
* **Action Item for EKF (`robot_localization`):** Replace zeros in `controllers.yaml` with the controller's recommended starting baseline `[0.001, 0.001, 0.001, 0.001, 0.001, 0.01]` so the EKF does not ignore or misweight wheel odometry.

---

## 5. Gazebo Ground Truth & Drift Comparison

Compare `/diff_drive_controller/odom` against the true physics pose in Gazebo Harmonic (`myWorld`).

```bash
# List Gazebo pose topics
gz topic -l | grep -i pose
# Output: /gui/camera/pose, /world/myWorld/dynamic_pose/info, /world/myWorld/pose/info

# Echo world entity poses (filter or inspect the "myrobot" block)
gz topic -e -t /world/myWorld/pose/info
```

### Drift Comparison Summary (`myrobot` vs. `/odom`)

| Source                                      | `x` (m)  | `y` (m)   | Notes                                                            |
| :------------------------------------------ | :------- | :-------- | :--------------------------------------------------------------- |
| **Wheel Odometry (`/odom`)**                | `1.0428` | `-0.2170` | Integrated from wheel encoders (drifts over time)                |
| **Gazebo Ground Truth (`myrobot`)**         | `1.0623` | `-0.2472` | True simulator pose (`~1.95 cm` X error, `~3.02 cm` Y error)     |

* **Key Hardware Offsets Observed in Gazebo World:**
  * `imu_link`: mounted at `z = +0.055 m` above `base_link`.
  * `left_wheel_link` / `right_wheel_link`: positioned at `y = +/-0.12 m` (`wheel_separation = 0.24 m`), `z = -0.02 m`.
  * `caster_wheel_link`: positioned at `x = -0.12 m`, `z = -0.04 m`.

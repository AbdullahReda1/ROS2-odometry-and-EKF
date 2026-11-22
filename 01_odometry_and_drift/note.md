# Session Notes: What Odometry Is and Why It Drifts

## 1. Core Concepts & Coordinate Frames

* **Odometry (Dead Reckoning):** Estimating a robot's current position and orientation solely by integrating incremental wheel movements over time without external world references.
* **Controller Behavior:** `diff_drive_controller` reads wheel encoder positions at every control cycle, computes the incremental step, publishes `nav_msgs/msg/Odometry`, and broadcasts the `odom -> base_link` transform.
* **Frame Distinction:**
  * `odom` **frame:** Starts at `(0, 0, 0)` where the robot spawns. Smooth and continuous locally, but **drifts unboundedly** over time.
  * `base_link` **frame:** Attached to the robot's chassis center.
  * `map` **frame:** Fixed global world frame anchored to external landmarks (LiDAR/SLAM); used later to correct `odom` drift.

---

## 2. Differential-Drive Dead-Reckoning Math

Given wheel displacements $d_{\text{left}}$, $d_{\text{right}}$ and wheel separation $L$ (`wheel_sep`):

$$d_{\text{centre}} = \frac{d_{\text{left}} + d_{\text{right}}}{2}$$

$$\Delta\theta = \frac{d_{\text{right}} - d_{\text{left}}}{L}$$

$$x \leftarrow x + d_{\text{centre}} \cos\left(\theta + \frac{\Delta\theta}{2}\right)$$

$$y \leftarrow y + d_{\text{centre}} \sin\left(\theta + \frac{\Delta\theta}{2}\right)$$

$$\theta \leftarrow \theta + \Delta\theta$$

* **Midpoint Approximation ($\theta + \Delta\theta / 2$):** Uses the average heading during the time step rather than the initial heading to better approximate a curved arc.
* **Heading Sensitivity:** Position updates depend on $\cos(\theta)$ and $\sin(\theta)$, so small angular errors bend the trajectory and compound over distance ($\text{Position Error} \approx \text{Distance} \times \Delta\theta_{\text{rad}}$).

---

## 3. Sources of Odometry Drift

* **Systematic (Fixed) Errors:** Incorrect `wheel_radius` (distance scaling error) or incorrect `wheel_separation` (turning angle error) in YAML configuration—repeatable and fixable via calibration.
* **Random (Non-Systematic) Errors:** Wheel slip, uneven terrain/bumps, and discretization/update-rate rounding—cannot be calibrated out and require sensor fusion (IMU, LiDAR/SLAM via EKF).

| Heading Error | Position Error after 1 m | Position Error after 10 m |
| :--- | :--- | :--- |
| $0.5^\circ$ | $0.9\text{ cm}$ | $8.7\text{ cm}$ |
| $1.0^\circ$ | $1.7\text{ cm}$ | $17.5\text{ cm}$ |
| $5.0^\circ$ | $8.7\text{ cm}$ | $87.0\text{ cm}$ |

---

## 4. Multi-Sensor Preview (Why EKF Is Needed)

| Source | Strengths | Weaknesses |
| :--- | :--- | :--- |
| **Wheel Odometry** | Fast, smooth short-term motion | Suffers from slip; drifts over distance |
| **IMU** | Accurate turning rate; unaffected by wheel slip | Double-integrating acceleration drifts rapidly |
| **LiDAR + SLAM** | Drift-free global pose in `map` | Higher CPU load; requires a map |

---

## 5. Lab Findings & Diagnostics (`01_odometry_and_drift`)

* **Active Controllers:** `diff_drive_controller` and `joint_state_broadcaster` are both `active`.
* **Topics & Message Types (ROS 2 Jazzy):**
  * Command topic: `/diff_drive_controller/cmd_vel` (`geometry_msgs/msg/TwistStamped`)
  * Odometry topic: `/diff_drive_controller/odom` (`nav_msgs/msg/Odometry`, `frame_id: odom`, `child_frame_id: base_link`)
* **Covariance Issue Discovered:**
  * Both `pose_covariance_diagonal` and `twist_covariance_diagonal` are currently `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]`, resulting in all-zero `36`-element covariance arrays.
  * **Fix for EKF Readiness:** Populate non-zero diagonal variances in `controllers.yaml` (recommended starting baseline from `ros2 param describe`: `[0.001, 0.001, 0.001, 0.001, 0.001, 0.01]`), because `robot_localization` ignores measurements with zero variance.
* **Measured Simulation Drift (`myrobot`):**
  * `/diff_drive_controller/odom` reported: `x = 1.0428 m`, `y = -0.2170 m`
  * Gazebo ground truth (`/world/myWorld/pose/info`) reported: `x = 1.0623 m`, `y = -0.2472 m`
  * Even in simulation with minimal slip, odometry diverged by **~2.0 cm in X** and **~3.0 cm in Y** (~3.6 cm Euclidean error), demonstrating that dead reckoning cannot detect its own error.

---

## 6. Essential ROS 2 & Gazebo Command Reference

```bash
# Check active controllers and topics
ros2 control list_controllers
ros2 topic list | grep -E "odom|cmd_vel"
ros2 topic info /diff_drive_controller/cmd_vel

# Drive robot using TwistStamped (ROS 2 Jazzy default)
ros2 topic pub -r 10 -t 30 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: auto, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"

# Inspect odometry pose and covariance
ros2 topic echo /diff_drive_controller/odom --field pose.pose.position --once
ros2 topic echo /diff_drive_controller/odom --field pose.covariance --once

# Inspect controller covariance parameters
ros2 param get /diff_drive_controller pose_covariance_diagonal
ros2 param describe /diff_drive_controller pose_covariance_diagonal

# Inspect Gazebo Harmonic ground-truth pose
gz topic -l | grep -i pose
gz topic -e -t /world/myWorld/pose/info
```

---

## 7. Common Traps to Avoid

* **Clock Mismatch:** Gazebo runs at roughly half real-time speed; always evaluate rates in simulation time (`use_sim_time: true`) rather than wall-clock time.
* **Message Type Mismatch:** Publishing `geometry_msgs/msg/Twist` instead of `TwistStamped` in Jazzy will fail silently unless `use_stamped_vel: false` is configured.
* **Parameter Mismatch:** Ensure `wheel_radius` and `wheel_separation` in `controllers.yaml` match the URDF geometry exactly.
* **TF Conflicts (Upcoming EKF Step):** Set `enable_odom_tf: false` on `diff_drive_controller` once `ekf_node` is added so only the EKF broadcasts `odom -> base_link`.

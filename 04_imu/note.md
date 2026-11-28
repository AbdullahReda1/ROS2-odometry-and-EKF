# The IMU

**Goal:** Understand what an Inertial Measurement Unit (IMU) measures, why it fixes the exact weak point of wheel odometry, why it cannot navigate alone, and how your simulated IMU behaves in ROS 2 Jazzy.

---

## 1. Core Concept: What an IMU Measures

Unlike wheel odometry (which counts external wheel rotations), an **IMU** senses motion internally without looking at the world or relying on wheel traction:

* **Gyroscope:** Measures angular velocity (turn rate in `rad/s`) around `x`, `y`, and `z`.
* **Accelerometer:** Measures linear acceleration (in `m/s²`) along `x`, `y`, and `z`.
* **Orientation (Quaternion):** Many IMUs (and Gazebo's IMU plugin) also publish a 3D orientation `(x, y, z, w)` computed by an onboard orientation filter.

### Why It Pairs with Wheel Odometry

* **Wheel odometry** drifts mainly in **heading (yaw)** because tires slip during turns.
* **The gyroscope** measures physical rotation directly inside the sensor chip—it is completely unaffected by wheel slip.
* **The EKF strategy:** Take **linear position/velocity** from wheel odometry and **yaw turn rate (`angular_velocity.z`)** from the IMU.

---

## 2. Why an IMU Cannot Work Alone (Integration Drift)

Because gyroscopes and accelerometers measure **rates of change**, getting angle or position requires integrating over time—causing tiny sensor bias/noise to accumulate rapidly:

| Measurement                                                                | Integration Steps                                                       | Error Growth                                        | Example Drift (after 60 s)                                   |
| :------------------------------------------------------------------------- | :---------------------------------------------------------------------- | :-------------------------------------------------- | :----------------------------------------------------------- |
| **Gyroscope (`angular_velocity.z`) $\rightarrow$ Heading**                 | Integrated **once** ($\theta = \int \omega \, dt$)                      | Linear with time ($\propto t$)                      | `0.1 °/s` bias $\rightarrow$ **`6°` error**                  |
| **Accelerometer (`linear_acceleration`) $\rightarrow$ Position**           | Integrated **twice** ($v = \int a \, dt$, $x = \int v \, dt$)           | Quadratic with time ($\propto \frac{1}{2} b t^2$)   | `0.01 m/s²` bias $\rightarrow$ **`18 m` error**              |

> **Key Takeaway:** Double integration makes accelerometer-based position unusable within seconds. Always fuse wheel odometry for position and use the IMU primarily for `angular_velocity.z` (and optionally `yaw`).

---

## 3. Inside `sensor_msgs/msg/Imu`

| Message Field               | Unit                     | Meaning                                   | Used for 2D Ground Robot?                    |
| :-------------------------- | :----------------------- | :---------------------------------------- | :------------------------------------------- |
| `header.frame_id`           | `string`                 | Sensor coordinate frame (`imu_link`)      | **Yes** (must exist in `/tf_static`)         |
| `orientation`               | Quaternion `(x,y,z,w)`   | 3D rotation estimate                      | Optional (for yaw; convert from quaternion)  |
| `angular_velocity.z`        | `rad/s`                  | Yaw turn rate around vertical Z-axis      | **Yes — Primary EKF input**                  |
| `angular_velocity.x, y`     | `rad/s`                  | Roll and pitch turn rates                 | No (2D planar robot)                         |
| `linear_acceleration.x`     | `m/s²`                   | Forward/backward acceleration             | Weak / noisy                                 |
| `linear_acceleration.z`     | `m/s²`                   | Vertical acceleration (`+9.81` at rest)   | No                                           |
| `*_covariance` (3 arrays)   | `float64[9]`             | `3×3` uncertainty matrices for EKF        | **Yes** (must be non-zero for EKF)           |

### Two Common Clarifications

1. **Why `linear_acceleration.z` reads `+9.81 m/s²` at rest:** An accelerometer measures the upward normal force of the ground pushing against gravity (`+1 g`), not free-fall acceleration. This is normal.
2. **Reading `orientation` as Roll, Pitch, Yaw:** Because `orientation` is a 4-number quaternion `(x, y, z, w)` rather than degrees/radians, either convert it with Euler conversion utilities or feed `angular_velocity.z` directly into the EKF.

---

## 4. Your Verified Robot Setup & Lab Results

### Hardware & Bridge Configuration

* **URDF Mounting (`robot.urdf`):** `imu_link` is attached to `base_link` via fixed joint `imu_joint` at `[0.000, 0.000, 0.055] m` with zero rotation (`rpy = [0, 0, 0]`).
* **Simulation Rate:** Configured at `100 Hz` sim time (`<update_rate>100</update_rate>`) with no `<noise>` block (ideal noise-free simulation sensor).
* **Gazebo Bridge (`bridge.yaml`):** Bridges Gazebo topic `imu` (`gz.msgs.IMU`) to ROS 2 topic `/imu` (`sensor_msgs/msg/Imu`).

### Measured Lab Diagnostics (`04_imu`)

* **Topic & Frame:** `/imu` (`sensor_msgs/msg/Imu`), `header.frame_id: imu_link`.
* **Observed Rate (`ros2 topic hz /imu`):** `~73.3 Hz` wall-clock time (reflecting `100 Hz` sim time).
* **At-Rest Readings:**
  * `orientation`: `(x ≈ 0.0, y ≈ 0.0, z ≈ 0.0, w ≈ 1.0)`
  * `linear_acceleration.z`: `9.81 m/s²`
  * `angular_velocity.z`: `~0.0 rad/s` (matches `/diff_drive_controller/odom` `twist.twist.angular.z`)
  * `*_covariance`: All 9 elements are currently `0.0` (note: real IMUs set `covariance[0] = -1` if a field is unsupported, and `robot_localization` requires positive diagonal values).
* **Recorded Rosbag (`imu_bag_0.mcap`):**
  * Duration: `17.47 s` | Total messages: `1,953`
  * `/imu`: `1,303` messages (`~74.6 Hz` wall-clock $\approx 100\text{ Hz}$ sim)
  * `/diff_drive_controller/odom`: `650` messages (`~37.2 Hz` wall-clock $\approx 50\text{ Hz}$ sim — exact 2:1 rate ratio)

---

## 5. Practical Command Reference

```bash
# 1. Verify bridged IMU topic and frame
ros2 topic list -t | grep imu
ros2 topic echo /imu --once --field header.frame_id

# 2. Check publish rate (wall-clock vs. 100 Hz sim time)
ros2 topic hz /imu

# 3. Inspect rest values and covariance
ros2 topic echo /imu --once --field orientation
ros2 topic echo /imu --once --field linear_acceleration
ros2 topic echo /imu --once --field angular_velocity
ros2 topic echo /imu --once --field orientation_covariance

# 4. Compare live yaw turn rate: IMU vs. Wheel Odometry
ros2 topic echo /imu --once --field angular_velocity.z
ros2 topic echo /diff_drive_controller/odom --once --field twist.twist.angular.z

# 5. Verify static TF mounting offset
ros2 run tf2_ros tf2_echo base_link imu_link

# 6. Record and replay bag data (note: ros2 bag play does not move Gazebo/RViz unless /tf is recorded)
ros2 bag record -o imu_bag --topics /imu /diff_drive_controller/odom
ros2 bag info imu_bag
ros2 bag play imu_bag
```

---

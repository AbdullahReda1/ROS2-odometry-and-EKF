# Session 4 Lab Log: Bridging & Verifying the IMU (`/imu`)

**Directory:** `/mnt/ubuntu_data/ROS2/ROS2-odometry-and-EKF/04_imu`
**ROS 2 Distro:** Jazzy | **Bag Format:** MCAP

---

## 1. Topic Discovery, Frame & Publish Rate

```bash
ros2 topic list -t | grep imu
# Output: /imu [sensor_msgs/msg/Imu]

ros2 topic echo /imu --once --field header.frame_id
# Output: imu_link

ros2 topic hz /imu
# Output: average rate: ~73.3 Hz (wall-clock time; configured at 100 Hz sim time in URDF)
```

* **Key Observation:** `ros2 topic hz` measures wall-clock seconds, so a `100 Hz` sim-time sensor reads lower (`~61–73 Hz`) when Gazebo runs below real-time speed.

---

## 2. IMU Readings at Rest vs. After Motion

| Field | At Rest (`sec: 179.61`) | After Driving / Turning | Notes |
| :--- | :--- | :--- | :--- |
| **`header.frame_id`** | `imu_link` | `imu_link` | Matches `gz_frame_id` in `robot.urdf`. |
| **`orientation` (`x, y, z, w`)** | `[0.0, 0.0, 0.0, 1.0]` | `[-0.033, 0.059, 0.486, 0.871]` | `w ≈ 1.0` at spawn; `z = 0.486, w = 0.871` corresponds to ~`58.3°` yaw rotation. |
| **`angular_velocity.z`** | `~0.0 rad/s` (`2.74e-18`) | `~0.0 rad/s` (sampled after stopping) | Ideal noiseless simulation IMU (`<noise>` tag omitted). |
| **`linear_acceleration.z`** | `+9.81 m/s²` | `+9.72 m/s²` | Measures the upward normal reaction force (`+1 g`) against gravity. |
| **`linear_acceleration.x`** | `~0.0 m/s²` (`1.46e-06`) | `-1.33 m/s²` | Captured during deceleration or slight chassis pitch tilt (`x = -0.033`). |
| **`*_covariance` (all 3)** | `[0.0, ..., 0.0]` (9 zeros) | `[0.0, ..., 0.0]` (9 zeros) | Zeroed by default in Gazebo; must be populated or overridden before EKF fusion. |

---

## 3. Gyro Yaw Rate vs. Wheel Odometry Yaw Rate Comparison

When sampled while stationary, both sources agree near zero (confirming zero bias in simulation):

```bash
ros2 topic echo /imu --once --field angular_velocity.z
# Output: -7.006e-13 rad/s  (second check: -1.250e-17 rad/s)

ros2 topic echo /diff_drive_controller/odom --once --field twist.twist.angular.z
# Output: -4.996e-16 rad/s  (second check: -1.729e-15 rad/s)
```

---

## 4. Static Mounting Transform Verification (`tf2_echo`)

```bash
ros2 run tf2_ros tf2_echo base_link imu_link
# Translation: [0.000, 0.000, 0.055]
# Rotation (RPY degree): [0.000, -0.000, 0.000]
```

* **Key Observation:** Confirms `imu_link` sits exactly `5.5 cm` above `base_link` with zero rotational offset, matching `<origin xyz="0.0 0.0 0.055" rpy="0.0 0.0 0.0"/>` in `robot.urdf`.

---

## 5. ROS 2 Bag Recording & Inspection (`imu_bag`)

Recorded both sensor streams for offline EKF testing without needing Gazebo running:

```bash
ros2 bag record -o imu_bag /imu /diff_drive_controller/odom
ros2 bag info imu_bag
```

| Bag Property | Value |
| :--- | :--- |
| **File & Storage** | `imu_bag_0.mcap` (`981.3 KiB`, `mcap` format) |
| **Duration** | `17.47 s` (wall time) |
| **Total Messages** | `1,953` |
| **`/imu` Count** | `1,303` messages (`~74.6 Hz` wall time ≈ `100 Hz` sim time) |
| **`/diff_drive_controller/odom` Count** | `650` messages (`~37.2 Hz` wall time ≈ `50 Hz` sim time) |

* **Ratio Check:** `1,303 / 650 ≈ 2.005` — confirming `/imu` publishes at **2× the rate** of `/diff_drive_controller/odom` (`100 Hz` vs. `50 Hz` in sim time).

---

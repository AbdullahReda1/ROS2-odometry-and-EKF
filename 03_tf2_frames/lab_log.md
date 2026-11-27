# TF2 Frames (`map`, `odom`, `base_link`)

**Working Directory:** `/mnt/ubuntu_data/ROS2/ROS2-odometry-and-EKF/03_tf2_frames`
**Robot Model:** `diff_drive_robot` (`myrobot`)

---

## 1. TF Tree Structure (`view_frames`)

Generated the live TF2 graph PDF (`robot_tf.pdf`) over a 5-second listening window:

```bash
ros2 run tf2_tools view_frames -o robot_tf
```

### Verified Frame Hierarchy & Update Rates

```text
odom  (Single Root)
└── base_link             [50.27 Hz | Dynamic on /tf]
    ├── imu_link          [Static   | At time 0.0 on /tf_static]
    ├── caster_wheel_link [Static   | At time 0.0 on /tf_static]
    ├── left_wheel_link   [19.77 Hz | Dynamic on /tf]
    └── right_wheel_link  [19.77 Hz | Dynamic on /tf]
```

| Child Frame           | Parent Frame  | Update Rate                        | Buffer Length | Transform Type                      |
| :-------------------- | :------------ | :--------------------------------- | :------------ | :---------------------------------- |
| `base_link`           | `odom`        | `50.271 Hz`                        | `3.879 s`     | Dynamic (`diff_drive_controller`)   |
| `left_wheel_link`     | `base_link`   | `19.767 Hz`                        | `3.440 s`     | Dynamic (`robot_state_publisher`)   |
| `right_wheel_link`    | `base_link`   | `19.767 Hz`                        | `3.440 s`     | Dynamic (`robot_state_publisher`)   |
| `imu_link`            | `base_link`   | `10000.0 Hz` *(static marker)*     | `0.000 s`     | Static (`robot_state_publisher`)    |
| `caster_wheel_link`   | `base_link`   | `10000.0 Hz` *(static marker)*     | `0.000 s`     | Static (`robot_state_publisher`)    |

> **Note:** `map` is absent from the tree, confirming REP-105 compliance prior to adding SLAM.

---

## 2. TF Topics, Publishers & QoS (`ros2 topic info`)

```bash
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
```

| Topic                    | Message Type               | Publishers                                     | Durability QoS                     | Responsibility                                                                              |
| :----------------------- | :------------------------- | :--------------------------------------------- | :--------------------------------- | :------------------------------------------------------------------------------------------ |
| **`/tf`**                | `tf2_msgs/msg/TFMessage`   | `diff_drive_controllerrobot_state_publisher`   | `TRANSIENT_LOCAL` / `VOLATILE`     | Publishes moving transforms (`odom -> base_link` and wheel joints).                         |
| **`/tf_static`**         | `tf2_msgs/msg/TFMessage`   | `robot_state_publisher` (only)                 | `TRANSIENT_LOCAL`                  | Broadcasts fixed URDF sensor/caster transforms once so late subscribers still receive them. |

---

## 3. Inspecting Live, Static & Chained Transforms (`tf2_echo`)

### A. Dynamic Transform: `odom -> base_link`

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

* **Initial Startup Behavior:** Printed `Waiting for transform odom -> base_link: Invalid frame ID "odom" ... frame does not exist` until the first `/tf` message arrived—expected behavior.
* **Observed Motion:**

| Sim Time (`s`)   | Translation `[x, y, z]` (`m`) | Yaw (`rad`) | Yaw (`deg`) | Status                                                             |
| :--------------- | :---------------------------- | :---------- | :---------- | :----------------------------------------------------------------- |
| `1287.39`        | `[1.075, 0.375, 0.000]`       | `0.775`     | `44.397°`   | Moving forward along `44.4°` heading                               |
| `1288.19`        | `[1.187, 0.484, 0.000]`       | `0.775`     | `44.397°`   | Reached target displacement (`+0.112 m` in X, `+0.109 m` in Y)     |
| `1289.50`        | `[1.187, 0.484, 0.000]`       | `0.775`     | `44.397°`   | Stationary                                                         |

### B. Static Transform: `base_link -> imu_link`

```bash
ros2 run tf2_ros tf2_echo base_link imu_link
```

* **Timestamp:** Always `At time 0.0` (standard for `/tf_static`).
* **Constant Offset:** Translation `[0.000, 0.000, 0.055]`, Rotation RPY `[0.000, 0.000, 0.000]`—matches `<origin xyz="0.0 0.0 0.055"/>` in `robot.urdf`.

### C. Chained Transform: `odom -> imu_link`

```bash
ros2 run tf2_ros tf2_echo odom imu_link
```

TF2 automatically chains `odom -> base_link` with `base_link -> imu_link`, carrying the `0.055 m` Z-height offset into the global `odom` frame as the robot turns and drives:

| Sim Time (`s`)   | Translation `[x, y, z]` (`m`) | Yaw (`rad`) | Yaw (`deg`) | Notes                                                    |
| :--------------- | :---------------------------- | :---------- | :---------- | :------------------------------------------------------- |
| `1465.11`        | `[1.494, 0.871, 0.055]`       | `1.275`     | `73.050°`   | Turning left while moving                                |
| `1465.95`        | `[1.515, 1.010, 0.055]`       | `1.559`     | `89.301°`   | Approaching `90°` heading                                |
| `1466.81`        | `[1.505, 1.118, 0.055]`       | `1.775`     | `101.695°`  | Completed turn (`101.7°`), driving forward               |
| `1468.43`        | `[1.454, 1.363, 0.055]`       | `1.775`     | `101.695°`  | Heading locked at `101.695°`; `z` stays `0.055 m`        |

---

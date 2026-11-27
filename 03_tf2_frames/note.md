# Session 3 Notes: TF2 Frames (`map`, `odom`, `base_link`)

## 1. Core TF2 Tree Rules

* **Coordinate Frame:** A 3D coordinate system (`x, y, z` + orientation) attached to a robot link or the environment.
* **Tree Structure (Strict Single-Parent Rule):**
  * Every frame has **at most one parent** and zero or more children.
  * Loops and multi-parent frames are forbidden so there is always **one unique path** between any two frames.
  * A healthy TF tree has **one root** (e.g., `odom` today, `map` once SLAM is active).

---

## 2. REP-105 Mobile Robot Frame Chain

Standard ROS mobile robots follow the **REP-105** frame hierarchy:

```text
map  -->  odom  -->  base_link
```

| Frame | Anchored To | Moves Smoothly? | Can Jump? | Drifts Over Time? | Primary Role |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **`map`** | Real-world environment | No | **Yes** | **No** | Global navigation, long-term goal planning, and drift correction. |
| **`odom`** | Robot spawn origin | **Yes** | **No** | **Yes** (slowly) | Smooth, continuous local velocity control without sudden jerks. |
| **`base_link`** | Robot chassis center | Moves with robot | No | N/A | Reference frame for mounted sensors (`imu_link`, `lidar_link`, `camera_link`). |

### How `map` and `odom` Work Together (Hiding Jumps)

TF2 computes the robot's global pose in the map by chaining (multiplying) the two transforms:

$$T_{\text{map} \to \text{base\_link}} = T_{\text{map} \to \text{odom}} \times T_{\text{odom} \to \text{base\_link}}$$

* **During normal driving:** `odom -> base_link` updates smoothly from wheel encoders (accumulating small slip drift), while `map -> odom` stays constant.
* **When SLAM recognizes a landmark:** SLAM shifts **only** `map -> odom` to cancel the accumulated drift.
* **Result:** The sudden correction ("jump") is absorbed inside `map -> odom`, while `odom -> base_link` remains completely smooth for local controllers.

---

## 3. Who Publishes Each Transform

| Transform Link | Published By (Session 3) | Published By (Later Sessions) | Topic | Update Behavior |
| :--- | :--- | :--- | :--- | :--- |
| `map -> odom` | *None* (`map` does not exist yet) | **SLAM** (e.g., `slam_toolbox`) | `/tf` | Low-rate global corrections (can jump). |
| `odom -> base_link` | **`diff_drive_controller`** | **`ekf_filter_node`** (`robot_localization`) | `/tf` | Continuous ~50 Hz local pose estimate. |
| `base_link -> imu_link` (fixed sensors) | **`robot_state_publisher`** (from URDF) | **`robot_state_publisher`** | `/tf_static` | Sent once (`TRANSIENT_LOCAL` QoS) to save bandwidth. |
| `base_link -> left/right_wheel_link` | **`robot_state_publisher`** (from `/joint_states`) | **`robot_state_publisher`** | `/tf` | Continuous ~20 Hz updates from wheel angles. |

* **Why `/tf` vs. `/tf_static`?** Static joints never move relative to `base_link`, so `/tf_static` publishes them once with `TRANSIENT_LOCAL` durability (late-joining nodes still receive them). Moving joints and odometry stream continuously on `/tf`.
* **`tf_frame_prefix`:** Empty string (`""`) in `controllers.yaml`, keeping frame names plain (`odom`, `base_link`).

---

## 4. Your Verified Robot TF Tree & Lab Results

```text
odom  (Root — published by diff_drive_controller @ ~50.3 Hz)
└── base_link
    ├── imu_link          [0.000,  0.000,  0.055]  (static, /tf_static)
    ├── lidar_link        [-0.050, 0.000,  0.080]  (static, /tf_static)
    ├── caster_wheel_link [-0.120, 0.000, -0.040]  (static, /tf_static)
    ├── left_wheel_link   [0.000,  0.120, -0.020]  (dynamic @ ~19.8 Hz, /tf)
    └── right_wheel_link  [0.000, -0.120, -0.020]  (dynamic @ ~19.8 Hz, /tf)
```

### Key Lab Verification Findings

1. **Single Healthy Root:** `ros2 run tf2_tools view_frames -o robot_tf` confirmed `odom` is the sole root frame; `map` is absent as expected prior to SLAM.
2. **Active Broadcasters:**
   * `/tf` has **2 publishers**: `diff_drive_controller` and `robot_state_publisher`.
   * `/tf_static` has **1 publisher**: `robot_state_publisher`.
3. **Chained Transform Verification (`odom -> imu_link`):**
   * `base_link -> imu_link` remained fixed at `z = 0.055 m` (`At time 0.0`).
   * Querying `tf2_echo odom imu_link` automatically combined `odom -> base_link` with `base_link -> imu_link`, reporting `z = 0.055 m` in the `odom` frame as the robot moved and rotated.

---

## 5. Essential TF2 CLI Commands

```bash
# Generate a PDF diagram of the active TF tree
ros2 run tf2_tools view_frames -o robot_tf

# Inspect active TF publishers and QoS profiles
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose

# Live-print the transform between any two frames (source -> target)
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_ros tf2_echo odom imu_link
```

---

## 6. Common Traps & Quick Fixes

| Trap | Cause | Fix |
| :--- | :--- | :--- |
| RViz2 error: `Frame [map] does not exist` | SLAM is not running yet, so no node publishes `map`. | Set RViz2 **Fixed Frame** to `odom` until SLAM is added. |
| Robot flickers or teleports back and forth | Two nodes publish `odom -> base_link` simultaneously. | Check `ros2 topic info /tf --verbose`; set `enable_odom_tf: false` when EKF takes over. |
| `tf2_echo` prints `frame does not exist` at startup | The TF listener buffer hasn't received the first message yet. | Wait 1–2 seconds for the buffer to populate. |
| Setting Fixed Frame to `base_link` in RViz2 | Locks the camera to the robot body so the world appears to move. | Use `odom` as Fixed Frame to observe robot motion relative to the ground. |

---

## 7. Quick Self-Test Review

1. **Why a tree instead of a free graph?** One parent per frame ensures a single, unambiguous transform path between any two frames.
2. **REP-105 chain:** `map -> odom -> base_link`.
3. **Why both `map` and `odom`?** `odom` provides smooth, jump-free motion for local control; `map` provides drift-free global coordinates by absorbing jumps in `map -> odom`.
4. **Who publishes `odom -> base_link`?** `diff_drive_controller` now; `ekf_filter_node` (`robot_localization`) in Session 6.
5. **Why is `/tf_static` separate?** Fixed joints never change, so broadcasting them once with `TRANSIENT_LOCAL` QoS saves network/CPU overhead.

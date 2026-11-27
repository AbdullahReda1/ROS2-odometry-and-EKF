
# TF2 you need: map, odom, base_link

Goal of this session: know the three frames of REP-105, who publishes each link, and how to read the TF tree.
You need this because the EKF (session 6) and SLAM (next chat) both work by publishing one of these links.

---

## 1. Simple explanation

A frame is a coordinate system attached to something. TF2 stores how frames sit relative to each other. It answers questions like "where is the lidar relative to the base?".

TF2 stores a **tree**. Each frame has exactly one parent. A frame may have many children. A frame can never have two parents.

Why a tree and not a free graph: with one parent, there is only one path between any two frames. So TF2 always gives one clear answer. If two parents were allowed, there could be two answers, and they would disagree.

REP-105 is the ROS rule for mobile robot frames. It names three frames and puts them in one chain:

```text
map  ->  odom  ->  base_link
```

| Frame         | Fixed in...        | Moves smoothly?  | Can jump? | Drifts?        |
| ------------- | ------------------ | ---------------- | --------- | -------------- |
| `map`         | The world          | No               | Yes       | No             |
| `odom`        | The world, roughly | Yes              | No        | Yes, slowly    |
| `base_link`   | The robot body     | Moves with robot | No        | Not applicable |

The reason for two world frames: they solve two different needs.

`odom` is smooth. Its position never jumps. So control and navigation code can use it safely. But it drifts, as you learned in session 1.

`map` does not drift. But it may jump when a new correction arrives (for example, when SLAM finds the robot is 10 cm off). Jumps are bad for control code.

So the design is clever. Odometry publishes `odom -> base_link` (smooth, drifting). Later, SLAM publishes `map -> odom` (the slow correction). The robot position in the map is the two links multiplied. A jump hides inside `map -> odom`, and `odom -> base_link` stays smooth.

---

## 2. Who publishes each link

| Link                                                   | Published by today                                 | Published by later | Topic          |
| ------------------------------------------------------ | -------------------------------------------------- | ------------------ | -------------- |
| `map -> odom`                                          | Nobody. The `map` frame does not exist yet         | SLAM (next chat)   | `/tf`          |
| `odom -> base_link`                                    | `diff_drive_controller`                            | EKF (session 6)    | `/tf`          |
| `base_link -> imu_link` and other fixed sensor links   | `robot_state_publisher` (from the URDF)            | Same               | `/tf_static`   |
| `base_link -> wheel links` and the camera pan joint    | `robot_state_publisher` (from `/joint_states`)     | Same               | `/tf`          |

Why two topics: `/tf_static` is for links that never move (fixed joints). Publishing them once saves traffic. `/tf` is for links that change (wheels, odometry).

Why `robot_state_publisher` can do the moving joints: it listens to `/joint_states` (made by `joint_state_broadcaster`). It combines those angles with the URDF and publishes the result.

Your controller file sets `tf_frame_prefix` to an empty string. As far as I know a prefix would rename the frames. Yours is empty, so the names stay plain: `odom` and `base_link`.

---

## 3. Your tree

This is the tree from your URDF (robot name `diff_drive_robot`). Check it with `view_frames` (section 5).

```text
odom
└── base_link
    ├── imu_link
    ├── lidar_link
    ├── camera_link           (via camera_pan_joint, revolute, moves)
    │   └── camera_optical_frame   (fixed)
    ├── left_wheel_link       (moves)
    ├── right_wheel_link      (moves)
    └── caster_wheel_link     (fixed)
```

Today `map` is not in the tree at all. That is correct. We add it only with SLAM.

---

## 4. How to read a TF tree

Read it from the root down. Each arrow is a transform "parent to child". It says where the child sits, in the parent's coordinates.

| Question                         | How to read it                                             |
| -------------------------------- | ---------------------------------------------------------- |
| Where is the robot?              | Look at `odom -> base_link`                                |
| Where is the lidar on the robot? | Look at `base_link -> lidar_link` (fixed, from the URDF)   |
| Which links are static?          | Those on `/tf_static`                                      |
| Is the tree broken?              | More than one root, or a missing link                      |

A healthy tree has **one root** and no loops. Two roots means two separate trees, and TF2 cannot connect them.

---

## 5. Practical (real commands)

Launch file for this session (same one as before, with RViz2 back):

| Component                                                          | Status                                          |
| ------------------------------------------------------------------ | ----------------------------------------------- |
| `gz sim`, `use_sim_time`, `/clock` bridge                          | Keep                                            |
| `robot_state_publisher`, `create`                                  | Keep                                            |
| Spawners: `joint_state_broadcaster`, `diff_drive_controller`       | Keep                                            |
| `teleop_twist_keyboard`                                            | Keep                                            |
| `rviz2` node                                                       | **New again** (back from session 1 and 2)       |
| Bridge `imu`, `scan`, `camera`; camera spawners; `rqt_gui`         | Drop                                            |

**Step 1. Start the robot.** Run your launch file as before.

**Step 2. Draw the tree.**

```bash
ros2 run tf2_tools view_frames
```

This makes a PDF of the tree in your current folder. It needs Graphviz installed. Not sure if yours has it. If it fails, run `sudo apt install graphviz`. Open the PDF. Check that `odom` is the root and `base_link` is under it.

**Step 3. See who publishes `/tf`.**

```bash
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
```

Look at the publisher node names. On `/tf` you should see `diff_drive_controller` and `robot_state_publisher`. On `/tf_static` you should see `robot_state_publisher` only.

**Step 4. Read one link.**

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link imu_link
```

The first changes when you drive. The second stays the same, because it is a fixed joint. At the start you may see a warning that the frame `odom` does not exist. That is normal: the first message has not arrived yet.

**Step 5. Check in RViz2.** Your `robot.rviz` already has what we need: Fixed Frame `odom`, a TF display, and an Odometry display on `/diff_drive_controller/odom` (it keeps the last 100 arrows, so you see the path). It also has an Image display and a LaserScan display. Turn those two off for now. Their topics are not bridged in this session, and the image is heavy for your slow PC.

1. Look at the TF axes. They should follow the robot.
2. Drive with teleop. The arrows and the robot should move together.

RViz2 shows a frame in the Fixed Frame field only when it exists. Try typing `map`. You will see an error. That is correct, because nobody publishes `map` yet.

**Step 6 (optional). Try the wrong frame.** Set Fixed Frame to `base_link`. The robot stays still and the world moves. This shows why we use `odom` as the fixed frame: it is the frame the robot moves in.

---

## 6. Common traps

| Trap                                         | Why it happens                    | What to do                                              |
| -------------------------------------------- | --------------------------------- | ------------------------------------------------------- |
| RViz2 error: frame `map` does not exist      | Nobody publishes `map` yet        | Use `odom` as Fixed Frame until SLAM                    |
| Robot jumps or flickers in RViz2             | Two nodes publish the same link   | Check publishers with `ros2 topic info /tf --verbose`   |
| TF display shows nothing                     | RViz2 not on sim time             | Keep `use_sim_time` on for all nodes                    |
| `tf2_echo` waits and warns                   | First message has not arrived     | Wait a few seconds                                      |
| Two roots in the tree                        | A link has no parent              | Fix the URDF joint or the missing publisher             |
| Using `odom` as a permanent world position   | `odom` drifts                     | Use `map` for long-term position (SLAM)                 |
| Mixing up `/tf` and `/tf_static`             | Fixed links are on `/tf_static`   | Look at both when a link is missing                     |

---

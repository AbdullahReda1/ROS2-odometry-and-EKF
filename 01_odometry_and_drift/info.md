
# info_1: What odometry is and why it drifts (Pass 1)

Goal of this session: understand what `/odom` really is, and why you cannot trust it for long.
You need this first. The EKF in session 5 only makes sense if you know what is wrong with the raw data.

---

## 1. Simple explanation

Odometry means "guess my position from my own motion". The robot does not look at the world. It only counts how far its wheels turned. From that count it works out how far it moved and how much it turned. Then it adds this to the last position.

This method has a name: **dead reckoning**. Sailors used it before GPS. They knew speed, heading and time. They added small steps to a start point.

The reason it works: each step is easy to compute. The reason it fails: each step has a small error, and the errors never go away. They only add up. After one minute the robot may be a few centimetres off. After ten minutes it may be metres off.

Your `diff_drive_controller` does this at every control cycle. It reads the wheel positions, computes the step, adds it, and publishes the result as `nav_msgs/msg/Odometry` plus the `odom -> base_link` transform.

One more key point. Odometry starts at zero where the robot was spawned. It is a position **relative to the start**, not a position in the world. That is why the frame is called `odom` and not `map`. You will meet this again in session 3.

---

## 2. The maths, in small steps

A two-wheel robot has a left wheel speed `v_l` and a right wheel speed `v_r`. The distance between the wheels is `L` (wheel separation). The wheel radius is `r`. The controller needs `r` and `L` to turn wheel angles into metres. If either number is wrong, every step is wrong.

```python
import math

def step(x, y, theta, d_left, d_right, wheel_sep):
    """One dead reckoning step.
    d_left, d_right = metres each wheel rolled since the last step.
    """
    d_centre = (d_left + d_right) / 2.0          # how far the middle moved
    d_theta  = (d_right - d_left) / wheel_sep    # how much the heading changed
    x     += d_centre * math.cos(theta + d_theta / 2.0)
    y     += d_centre * math.sin(theta + d_theta / 2.0)
    theta += d_theta
    return x, y, theta
```

Why we use `theta + d_theta / 2`: during the step the robot is turning. The middle heading is a better guess for the direction of travel than the start heading.

Why heading is the weak point: the position is built from `cos(theta)` and `sin(theta)`. A small error in `theta` bends the whole path. Then every later step goes in a slightly wrong direction.

---

## 3. Why it drifts

| Cause                  | What goes wrong                          | Kind               |
| ---------------------- | ---------------------------------------- | ------------------ |
| Wrong wheel radius     | Distance is always too long or too short | Fixed (systematic) |
| Wrong wheel separation | Turns are always too big or too small    | Fixed (systematic) |
| Wheel slip             | Wheel turns, robot does not move         | Random             |
| Bumps and uneven floor | Wheel lifts or the robot tilts           | Random             |
| Rounding, update rate  | Tiny motion is lost in each step         | Small, random      |

Fixed errors repeat the same way every time. You can measure them and fix them with calibration. Random errors change every time. You cannot remove them. You can only reduce them by adding another sensor. This is the reason for the EKF later.

How heading error grows into position error:

| Heading error | Error after 1 m | Error after 10 m |
| ------------- | --------------- | ---------------- |
| 0.5 deg       | 0.9 cm          | 8.7 cm           |
| 1 deg         | 1.7 cm          | 17.5 cm          |
| 5 deg         | 8.7 cm          | 87 cm            |

Rule of thumb: position error is about distance times heading error in radians. The error grows with distance, so long runs hurt most.

---

## 4. Which sensors can fix it (preview only)

You do not need the details yet. This table shows why we will mix sensors.

| Source         | Good at                  | Bad at                      | Session |
| -------------- | ------------------------ | --------------------------- | ------- |
| Wheel odometry | Smooth, fast, short term | Slip, drifts over distance  | 2       |
| IMU            | Turning rate, no slip    | Position (drifts very fast) | 4       |
| Lidar + SLAM   | Absolute position in map | Needs a map, uses more CPU  | 7       |

The idea: each sensor is wrong in a different way. Mixing them cancels some of the errors.

---

## 5. Practical (real commands)

Your Gazebo runs at about half real time. So everything below looks slow in wall-clock time. That is normal. Do not judge rates by your watch.

I do not know your package name or topic names yet. Replace the `<...>` parts. Find the real names with step 2.

**Step 1. Start the robot.**

```bash
ros2 launch <your_pkg> bringup.launch.py
```

**Step 2. Find the odometry and velocity topics.** Open a second terminal.

```bash
ros2 topic list | grep -E "odom|cmd_vel"
ros2 control list_controllers
```

The controller should show as `active`. The topic names depend on your `controllers.yaml` and your launch remaps. Check the message type of the command topic before you publish to it:

```bash
ros2 topic info <cmd_vel_topic>
```

**Step 3. Read one odometry message.**

```bash
ros2 interface show nav_msgs/msg/Odometry
ros2 topic echo <odom_topic> --once
```

Look at three things. The `frame_id` (should be `odom`). The `child_frame_id` (should be `base_link`). The pose, which should be near zero at the start.

**Step 4. Drive the robot a little.** Use the type you found in step 2. In Jazzy the controller command is, as far as I know, `geometry_msgs/msg/TwistStamped`. Not sure for your setup, so verify with `ros2 topic info`.

```bash
ros2 topic pub -r 10 -t 30 <cmd_vel_topic> geometry_msgs/msg/TwistStamped \
  "{header: auto, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

If the robot does not move, I am not sure why on your setup. Check the topic type again, and check that the controller is `active`.

**Step 5. Watch the pose change.**

```bash
ros2 topic echo <odom_topic> --field pose.pose.position
```

**Step 6. Look at the covariance.** This is the controller telling you how much to trust its own numbers.

```bash
ros2 topic echo <odom_topic> --field pose.covariance --once
```

I think `diff_drive_controller` has parameters called `pose_covariance_diagonal` and `twist_covariance_diagonal`. Not sure. Check in your `controllers.yaml`, or with:

```bash
ros2 param list | grep -i covariance
```

**Step 7 (optional). The key lesson.** Drive forward, then drive back by the same amount. Odom will say you are near zero. But the real robot in Gazebo may not be. Odom cannot see its own error. That is the whole problem with dead reckoning.

To see the true pose in Gazebo, list the Gazebo topics and look for a pose topic:

```bash
gz topic -l | grep -i pose
```

Not sure which pose topic your world publishes. Check the list first.

---

## 6. Common traps

| Trap                                                   | Why it happens                              | What to do                                    |
| ------------------------------------------------------ | ------------------------------------------- | --------------------------------------------- |
| Thinking `/odom` is the true position                  | It is a guess, relative to the start        | Treat it as smooth but drifting               |
| Expecting zero drift in simulation                     | Sim has little slip, so drift looks small   | Real robots drift far more                    |
| Wrong `wheel_radius` or `wheel_separation` in YAML     | Controller uses them for every step         | Match them to the URDF                        |
| Judging topic rates with a wall clock                  | Gazebo runs at half real time               | Think in sim time, use `use_sim_time`         |
| Publishing the wrong message type to cmd_vel           | Topic type is `Twist` or `TwistStamped`     | Run `ros2 topic info` first                   |
| Confusing `odom` frame with `map` frame                | Both look like "position"                   | `odom` drifts, `map` does not (session 3)     |

---

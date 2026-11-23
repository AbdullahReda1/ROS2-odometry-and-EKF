
# Wheel odometry with diff_drive_controller (Pass 1)

Goal of this session: know how your controller turns wheel motion into `/odom` and `odom -> base_link`, and which settings matter.
You need this because the EKF later takes this data as one of its inputs.

---

## 1. Simple explanation

In session 1 we did dead reckoning by hand. Now we see who does it for you.

`diff_drive_controller` is a `ros2_control` controller. It does two jobs.

Job one: it takes a velocity command (`cmd_vel`) and turns it into a speed for each wheel. This is the "drive" part.

Job two: it reads the wheel joints back, does the dead reckoning step from session 1, and publishes the result. This is the "odometry" part. This session is about job two.

Why it lives in the controller: the controller already reads the wheel joints at every control cycle. So it has the freshest wheel data. Doing odometry anywhere else would add delay.

The data path in your simulation:

```text
Gazebo wheel joints
      |  (gz_ros2_control hardware interface)
      v
state interfaces: position, velocity
      |
      v
diff_drive_controller  --->  odometry topic (nav_msgs/Odometry)
                       --->  /tf  (odom -> base_link)
```

The reason there is a hardware interface in between: `ros2_control` never talks to Gazebo directly. It talks to named interfaces. In simulation the plugin fills them from Gazebo. On a real robot, your motor driver fills them. The controller code stays the same. This is why you can move from sim to a real robot without rewriting it.

---

## 2. What the controller reads and writes

| Direction | What                    | Type                      | Note                                     |
| --------- | ----------------------- | ------------------------- | ---------------------------------------- |
| Reads     | Wheel joint position    | State interface           | Used when `position_feedback` is true    |
| Reads     | Wheel joint velocity    | State interface           | Used when `position_feedback` is false   |
| Reads     | `cmd_vel` command       | Topic                     | Twist or TwistStamped, check yours       |
| Writes    | Wheel velocity commands | Command interface         | One per wheel joint                      |
| Writes    | Odometry                | `nav_msgs/msg/Odometry`   | Pose, twist and covariance               |
| Writes    | `odom -> base_link`     | TF on `/tf`               | Can be switched off (see section 5)      |

Position or velocity feedback? Position is better for odometry. Reason: position is already the sum of all motion, so one missed sample loses nothing. Velocity must be multiplied by time, so a late or jittery sample gives a wrong step. As far as I know `position_feedback` defaults to true. Not sure for your version. Check it in section 6.

---

## 3. The settings that decide odometry quality

These are the parameter names I am confident about. Values below are examples from me. Use your own from `controllers.yaml`.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names:  ["left_wheel_joint"]     # example name
    right_wheel_names: ["right_wheel_joint"]    # example name
    wheel_separation: 0.30                      # metres, example
    wheel_radius: 0.05                          # metres, example
    odom_frame_id: odom
    base_frame_id: base_link
    enable_odom_tf: true
```

| Parameter                                   | What it does                         | Why it matters                               |
| ------------------------------------------- | ------------------------------------ | -------------------------------------------- |
| `wheel_separation`                          | Distance between the two wheels      | Wrong value gives wrong turning angle        |
| `wheel_radius`                              | Wheel size                           | Wrong value gives wrong distance             |
| `left_wheel_names`, `right_wheel_names`     | Joint names to read and command      | Must match the URDF joint names exactly      |
| `odom_frame_id`, `base_frame_id`            | Frame names in the message and TF    | Must match what the rest of your system uses |
| `enable_odom_tf`                            | Publish `odom -> base_link` or not   | Only one node may publish it (see section 5) |

Why `wheel_radius` and `wheel_separation` are in the YAML and also in the URDF: as far as I know the controller reads them from its own parameters, not from the URDF. So the same number lives in two places. If you change one, change the other. This is a very common source of silent errors.

More parameters exist for covariance, rates, timeouts and calibration multipliers. Not now. We use them in later passes. To see all of them for your version:

```bash
ros2 param list /diff_drive_controller
```

---

## 4. How the step is computed

You already know the maths from session 1. Two details are new.

First, the controller needs a time step. It uses the time between two control cycles. If Gazebo runs slow, the time step is in sim time, so the maths stays correct. This is why `use_sim_time` must be true for every node in a simulation.

Second, as far as I know the controller uses a small integration trick (a second-order Runge-Kutta step) when the robot turns, and an exact step when it goes straight. Not sure about the exact code in your version. The reason for the trick is the same as `theta + d_theta / 2` in session 1: the robot turns during the step.

---

## 5. One publisher for `odom -> base_link`

Today the controller publishes `odom -> base_link`. That is fine. It is the only one.

In session 6 the EKF will also be able to publish `odom -> base_link`. If both publish, TF gets two sources for the same transform. The result is a robot that jumps back and forth in RViz2.

The rule: **one publisher per transform**. When the EKF takes over, you set `enable_odom_tf` to false in the controller. We do this in session 6. Do not change it now.

---

## 6. Practical (real commands)

Gazebo runs at about half real time, so rates in wall-clock time will look half as fast. That is normal.

Replace `<...>` with your names. Send me your four files and I can fill them in for you.

**Step 1. Start the robot and check the controller.**

```bash
ros2 launch <your_pkg> bringup.launch.py
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

You should see `diff_drive_controller` as `active`. In the interface list, each wheel joint should have a velocity command interface and position and velocity state interfaces.

**Step 2. Read the controller parameters.** The node name is the controller name from your YAML.

```bash
ros2 param get /diff_drive_controller wheel_radius
ros2 param get /diff_drive_controller wheel_separation
ros2 param get /diff_drive_controller enable_odom_tf
ros2 param get /diff_drive_controller position_feedback
```

If `position_feedback` is not found, your version may not have it. Not sure. Use `ros2 param list /diff_drive_controller` to see what exists.

**Step 3. Compare with the URDF.** Open `robot.urdf` and find the wheel cylinder radius and the distance between the two wheel joint origins. They must match the numbers from step 2.

**Step 4. Check the frames and the publish rate.**

```bash
ros2 topic echo <odom_topic> --once --field header.frame_id
ros2 topic echo <odom_topic> --once --field child_frame_id
ros2 topic hz <odom_topic>
```

Expect `odom` and `base_link`. The rate will look lower than your control rate. Reason: `hz` counts wall-clock time, and your sim runs at about half speed.

**Step 5. Watch the transform.**

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Drive the robot a little (command from session 1, step 4). The numbers should change. We explain TF properly in session 3.

---

## 7. Common traps

| Trap                                            | Why it happens                               | What to do                                    |
| ----------------------------------------------- | -------------------------------------------- | --------------------------------------------- |
| YAML radius or separation differs from the URDF | The same number is stored in two places      | Compare them every time you edit one          |
| Wheel joint names in YAML do not match the URDF | The controller finds joints by name          | Copy names from the URDF, do not retype       |
| Controller loaded but not `active`              | Hardware interface missing or wrong name     | Run `ros2 control list_hardware_interfaces`   |
| Robot moves, odom stays at zero                 | Wrong `odom_frame_id` or joint names         | Echo the odom topic, check frame ids          |
| Odom topic name is not `/odom`                  | Controller topics are namespaced by its name | Use `ros2 topic list`, or remap in launch     |
| Two nodes publish `odom -> base_link`           | Controller and EKF both publish              | Turn off one (session 6)                      |
| Nodes mix sim time and wall time                | `use_sim_time` missing on one node           | Set it true on every node in the sim          |

---

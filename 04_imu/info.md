
# The IMU

Goal of this session: know what your IMU measures, what it is good at, and why it cannot work alone.
You need this because in session 5 and 6 the EKF will mix the IMU with wheel odometry.

---

## 1. Simple explanation

An IMU (inertial measurement unit) feels motion from the inside. It does not look at the world and it does not count wheel turns. It has two parts:

- A **gyroscope**. It measures how fast the robot turns (rad/s).
- An **accelerometer**. It measures acceleration (m/s²).

Your Gazebo IMU also gives an **orientation** (a quaternion). In a real IMU this comes from a filter inside the chip. Not sure how Gazebo makes it. Check it in step 4.

Why we want it: in session 1 you saw that wheel odometry is weakest in **heading**. Wheels slip when the robot turns. A gyroscope does not care about slip. It feels the turn directly. So the IMU fixes the exact weak point of the wheels.

Why it cannot work alone: to get angle you add up the gyro rate over time. To get position you add up acceleration twice. Every small error is added up too. For position this goes very bad, very fast (section 3).

Your IMU in the URDF: link `imu_link`, fixed joint at z = 0.055 above `base_link`, no rotation. It publishes `imu` at 100 Hz (sim time). It has no `<noise>` tag, so it is perfect in the sim. A real IMU is never perfect.

---

## 2. What the message holds

The ROS message is `sensor_msgs/msg/Imu`.

| Field                       | Unit       | Meaning                                | Useful for your robot?           |
| --------------------------- | ---------- | -------------------------------------- | -------------------------------- |
| `orientation`               | quaternion | Rotation of the IMU                    | Yes, for heading (yaw)           |
| `angular_velocity.z`        | rad/s      | Turn rate around the vertical axis     | **Yes, the best one**            |
| `angular_velocity.x, y`     | rad/s      | Roll and pitch rate                    | No, the robot stays flat         |
| `linear_acceleration.x`     | m/s²       | Forward acceleration                   | Weak, noisy                      |
| `linear_acceleration.z`     | m/s²       | Up. Reads about +9.8 at rest (gravity) | No                               |
| `*_covariance` (3 fields)   | varies     | How much to trust each part            | Yes, EKF reads these (session 5) |

Why `linear_acceleration.z` is +9.8 at rest: an accelerometer measures the push of the support, not the fall. A table pushes up on the robot with 1 g. So it reads +9.8 up. This is normal and not an error.

Covariance rule in this message, as far as I know: if a part is not given, its first covariance value is set to -1. Not sure how the bridge fills it. Check it in step 4.

---

## 3. Wheel odometry versus IMU

| Question                | Wheel odometry          | IMU                        |
| ----------------------- | ----------------------- | -------------------------- |
| What it measures        | Wheel rotation          | Turn rate and acceleration |
| Affected by wheel slip? | Yes                     | No                         |
| Heading                 | Drifts when wheels slip | Good in the short term     |
| Position                | Good in the short term  | Very bad (adds up twice)   |
| Rate in your sim        | 50 Hz (sim time)        | 100 Hz (sim time)          |
| Long-term drift         | Yes                     | Yes, from bias             |

How fast the IMU position error grows. These numbers are examples from me, not from your robot.

| Constant error                  | After 10 s | After 60 s |
| ------------------------------- | ---------- | ---------- |
| Accel bias 0.01 m/s², position  | 0.5 m      | 18 m       |
| Gyro bias 0.1 deg/s, heading    | 1 deg      | 6 deg      |

Reason for the big gap: position adds up twice. Error grows like time squared. Heading adds up once. Error grows like time. This is why we only take the **yaw rate** from the IMU, and let wheels give the position.

---

## 4. The bridge entry

The IMU message must cross from Gazebo to ROS. Add this entry back to `bridge.yaml`. It is the same text you already have.

```yaml
- ros_topic_name: "imu"
  gz_topic_name: "imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
```

Why it is off in sessions 1 to 3: every bridged message costs a little CPU. We switch it on only when we need it.

Your world file already has both plugins the IMU needs (`Imu` and `Sensors`). Nothing to change there.

---

## 5. Practical (real commands)

Launch file for this session: same as session 3. The only change is the bridge file.

| Component                                                           | Status              |
| ------------------------------------------------------------------- | ------------------- |
| Everything kept in session 3 (with `rviz2`)                         | Keep                |
| Bridge `imu`                                                        | **New again**       |
| Bridge `scan`, `camera/image_raw`; camera spawners; `rqt_gui`       | Drop                |

**Step 1. Add the `imu` entry to `bridge.yaml` and start the robot.** Run your launch file.

**Step 2. Find the topic and its rate.**

```bash
ros2 topic list -t | grep imu
ros2 topic hz /imu
```

I expect `/imu`, because the bridge name has no slash. Not sure. If it is different, use the name from the list. The rate may look near 50 Hz, not 100. Reason: your Gazebo runs at about half real time, and `hz` counts wall time.

**Step 3. Check the frame.**

```bash
ros2 topic echo /imu --once --field header.frame_id
```

I expect `imu_link`. This comes from the sensor setup in the URDF. Confirm it.

**Step 4. Read it at rest.**

```bash
ros2 topic echo /imu --once --field orientation
ros2 topic echo /imu --once --field linear_acceleration
ros2 topic echo /imu --once --field angular_velocity
ros2 topic echo /imu --once --field orientation_covariance
```

Check three things. The orientation is near `x 0, y 0, z 0, w 1` (the robot spawns facing forward). The acceleration z is about 9.8. The angular velocity is near 0. Because there is no noise, the numbers will be very clean.

**Step 5. Turn the robot and compare two sources.** Use teleop to turn in place. While it turns, read both:

```bash
ros2 topic echo /imu --once --field angular_velocity.z
ros2 topic echo /diff_drive_controller/odom --once --field twist.twist.angular.z
```

Both should be close. In the sim the wheels do not slip much, so they agree. On a real robot with slip they would not agree. That gap is the reason we fuse them.

**Step 6. Check the fixed link.**

```bash
ros2 run tf2_ros tf2_echo base_link imu_link
```

You should see a fixed translation of about z = 0.055 and no rotation.

**Step 7 (optional). Record a short bag with your own name.** The `-o` option names the output folder.

```bash
ros2 bag record -o imu_test /imu /diff_drive_controller/odom
ros2 bag info imu_test
```

Press `Ctrl+C` to stop. Drive a short loop while it records. We can look at this data later without running Gazebo, which saves your slow PC.

---

## 6. Common traps

| Trap                                            | Why it happens                                     | What to do                                     |
| ----------------------------------------------- | -------------------------------------------------- | ---------------------------------------------- |
| `/imu` is empty                                 | The `imu` entry is missing in `bridge.yaml`        | Add it and restart                             |
| Still empty after adding it                     | `Imu` or `Sensors` plugin missing in the world     | Check `myWorld.sdf`                            |
| Thinking `+9.8` on z is a bug                   | The accelerometer feels the support, not gravity   | It is correct at rest                          |
| Using acceleration to get position              | Adding up twice grows error like time squared      | Take position from wheels, not the IMU         |
| Reading orientation as roll, pitch, yaw numbers | It is a quaternion                                 | Convert it, or use the gyro z rate             |
| Trusting the sim IMU as real                    | The sim IMU has no noise                           | A real IMU needs noise and covariance tuning   |
| Wrong `frame_id` on the IMU                     | The sensor frame is set in the URDF                | Check it with `echo --field header.frame_id`   |
| Judging the rate with a wall clock              | Gazebo runs at half real time                      | Think in sim time                              |

---

# ROS2-odometry-and-EKF
In ROS 2, fusing odometry with an Extended Kalman Filter is handled by the standard robot_localization package using its ekf_node. It combines noisy sensor streams (like wheel encoders and an IMU) into a smooth, continuous pose estimate and publishes the required coordinate transforms for navigation stacks like Nav2.

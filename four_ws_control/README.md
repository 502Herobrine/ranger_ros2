# four_ws_control

This ROS 2 package is adapted from AgileX Robotics' official
[`ugv_gazebo_sim`](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/humble/ranger/four_ws_control)
`humble` branch at commit `8bf6a52a3139969d27548622e6144a90c2b80894`.

Body-frame `/cmd_vel` is converted with rigid-body wheel velocities
`(vx - wz*y, vy + wz*x)`, including lateral, reverse and combined motions.
Steering angles stay within ±π/2 by reversing wheel rotation. Output order is
front left, front right, rear left, rear right; wheel speeds are in rad/s.
The controller topics and joystick buttons (0: translation, 4: longitudinal/yaw,
5: pivot) are unchanged.

`/joint_states` must contain finite positions for all four steering joints in each
message. Both V3 names (`fl/fr/rl/rr_steering_joint`) and composite robot names
(`front_left/front_right/rear_left/rear_right_steering_joint`) are supported. Wheel rotation is
blocked until the maximum steering error is below 0.2 rad. Stopping retains the
last steering angles. If velocity commands or joint feedback stop for more than
`cmd_vel_timeout` (default 0.5 seconds on the node clock), wheel speeds become zero.
Publish commands continuously, for example with keyboard parameter
`repeat_rate:=10.0`. Use `use_sim_time:=true` with Gazebo.

Run the package regression checks in a sourced ROS 2 Humble workspace:

```bash
colcon build --symlink-install --packages-select four_ws_control
colcon test --packages-select four_ws_control --event-handlers console_direct+
colcon test-result --test-result-base build/four_ws_control --verbose
```

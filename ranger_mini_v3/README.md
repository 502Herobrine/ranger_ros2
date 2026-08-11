# Ranger Mini V3 description for ROS 2

This package ports AgileX Robotics' official ROS 1 `ranger_mini_v3` description
package to ROS 2 Humble. The xacro, mesh, CSV, and joint-name resources come
from upstream commit `27633a956c845903ee630538afeb17fe70afdd84` of
`agilexrobotics/ugv_gazebo_sim`.

The ROS 2 port replaces catkin with ament, converts the XML launch files to
Python launch files, updates the RViz configuration for RViz2, and integrates
Gazebo Classic with `gazebo_ros2_control`. A massless `base_footprint` root link
is added for KDL compatibility and ground alignment. The four-wheel-steering
controller is reused from AgileX Robotics' official `ugv_gazebo_sim` Humble
branch and adapted to the Mini V3 geometry and joint names.

```bash
ros2 launch ranger_mini_v3 display.launch.py
ros2 launch ranger_mini_v3 gazebo.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

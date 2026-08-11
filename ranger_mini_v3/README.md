# Ranger Mini V3 description for ROS 2

This package ports AgileX Robotics' official ROS 1 `ranger_mini_v3` description
package to ROS 2 Humble. The xacro, mesh, CSV, and joint-name resources come
from upstream commit `27633a956c845903ee630538afeb17fe70afdd84` of
`agilexrobotics/ugv_gazebo_sim`.

The ROS 2 port replaces catkin with ament, converts the XML launch files to
Python launch files, updates the RViz configuration for RViz2, and spawns the
model in Gazebo Classic through `gazebo_ros/spawn_entity.py`. A massless
`base_footprint` root link is added for KDL compatibility and ground alignment;
the official meshes and movable-joint structure are unchanged. The bundled
joint-name YAML is updated to match the eight joints in the V3 xacro.

```bash
ros2 launch ranger_mini_v3 display.launch.py
ros2 launch ranger_mini_v3 gazebo.launch.py
```

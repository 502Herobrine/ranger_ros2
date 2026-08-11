# Ranger ROS 2（Ranger Mini V3 仿真扩展）

本仓库基于 AgileX Robotics 官方
[`agilexrobotics/ranger_ros2`](https://github.com/agilexrobotics/ranger_ros2)
仓库的 `humble` 分支开发，保留了 Ranger 系列底盘的 ROS 2 驱动、消息定义、
CAN 配置脚本和实车启动方式。

在官方驱动的基础上，本仓库新增了 Ranger Mini V3 的 ROS 2 描述与仿真支持：

- 将 AgileX 官方 ROS 1 `ranger_mini_v3` 描述包迁移到 ROS 2 Humble；
- 加入官方 URDF/Xacro、Ranger Mini V3 DAE 网格和 RViz2 配置；
- 增加 RViz2 模型展示 launch；
- 增加 Gazebo Classic 仿真 launch 和 `gazebo_ros2_control` 接口；
- 复用官方 `ugv_gazebo_sim` Humble 分支的 `four_ws_control`，适配 V3 后可直接响应 `/cmd_vel`；
- 增加 `base_footprint`，使 TF 树、KDL 解析和地面对齐更合理；
- 修正 V3 模型的四个转向关节和四个车轮关节配置。

Ranger Mini V3 的原始模型资源来自 AgileX 官方
[`ugv_gazebo_sim`](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/ranger_mini/ranger_mini_v3)
仓库。本仓库是个人维护的扩展版本，并非 AgileX 官方发布仓库。

## 支持的底盘

- Ranger Mini V1.0

  <img src="./docs/ranger_mini_v1.png" width="350" alt="Ranger Mini V1" />

- Ranger Mini V2.0 / V3.0

  <img src="./docs/ranger_mini_v2.png" width="350" alt="Ranger Mini V2/V3" />

- Ranger

  <img src="./docs/ranger.png" width="300" alt="Ranger" />

## 仓库结构

| ROS 2 包 | 用途 |
| --- | --- |
| `ranger_base` | Ranger 底盘驱动，连接 CAN、发布底盘状态并接收 `/cmd_vel` |
| `ranger_bringup` | 实车启动、CAN 配置脚本和 V3 RViz2 快捷入口 |
| `ranger_msgs` | Ranger 自定义消息 |
| `ranger_mini_v3` | 新增的 V3 URDF/Xacro、网格、RViz2 和 Gazebo launch |
| `four_ws_control` | 从 AgileX 官方 Humble 仿真包迁入并适配 V3 的四轮转向控制节点 |

## 环境

本扩展版本按以下环境开发和验证：

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11（通过 `gazebo_ros`）

以下命令默认工作区为 `~/ranger_ws`。如果使用其他路径，请同步替换命令中的
工作区路径。

## 安装与编译

### 1. 安装依赖

```bash
sudo apt update
sudo apt install -y \
  libasio-dev \
  libboost-all-dev \
  can-utils \
  ros-humble-teleop-twist-keyboard \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-xacro \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  python3-numpy
```

### 2. 克隆 SDK 和本仓库

与官方驱动的安装方式相同，需要同时克隆 `ugv_sdk` 和 `ranger_ros2`：

```bash
mkdir -p ~/ranger_ws/src
cd ~/ranger_ws/src

git clone https://github.com/agilexrobotics/ugv_sdk.git
git clone -b humble https://github.com/502Herobrine/ranger_ros2.git
```

### 3. 安装缺失依赖并编译

```bash
cd ~/ranger_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

每次打开新终端后，都需要重新加载环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ranger_ws/install/setup.bash
```

也可以将第二行加入 `~/.bashrc`。

## 实车使用

### 1. 配置 CAN-To-USB

首次使用 CAN-To-USB 适配器时运行：

```bash
sudo bash ~/ranger_ws/src/ranger_ros2/ranger_bringup/scripts/setup_can2usb.bash
```

该脚本会加载 `gs_usb` 内核模块、以 500 kbit/s 启动 `can0`，并安装
`can-utils`。

设备重新插拔或计算机重新启动后，运行：

```bash
sudo bash ~/ranger_ws/src/ranger_ros2/ranger_bringup/scripts/bringup_can2usb.bash
```

检查 CAN 接口和底盘数据：

```bash
ip -details link show can0
candump can0
```

如果 `candump` 能持续收到数据，说明 CAN 通信正常。

### 2. 启动底盘驱动

Ranger Mini V3：

```bash
ros2 launch ranger_bringup ranger_mini_v3.launch.py
```

其他型号：

```bash
# Ranger
ros2 launch ranger_bringup ranger.launch.py

# Ranger Mini V1
ros2 launch ranger_bringup ranger_mini_v1.launch.py

# Ranger Mini V2
ros2 launch ranger_bringup ranger_mini_v2.launch.py
```

使用其他 CAN 接口时，可以覆盖 `port_name`：

```bash
ros2 launch ranger_bringup ranger_mini_v3.launch.py port_name:=can1
```

### 3. 键盘控制

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> [!WARNING]
> 该命令会向 `/cmd_vel` 发布运动指令，实车可能立即移动。测试前请架空车轮或
> 清空周围区域，并确保遥控器、急停和操作人员均处于安全状态。

## Ranger Mini V3 模型与仿真

仿真功能不需要连接 CAN 或启动底盘驱动。

### RViz2 展示官方 V3 模型

推荐直接启动描述包：

```bash
ros2 launch ranger_mini_v3 display.launch.py
```

也可以使用 `ranger_bringup` 中的快捷入口：

```bash
ros2 launch ranger_bringup v3_display.launch.py
```

默认会启动：

- `joint_state_publisher_gui`
- `robot_state_publisher`
- RViz2

在没有图形界面的环境中，可以只检查模型和 TF：

```bash
ros2 launch ranger_mini_v3 display.launch.py gui:=false rviz:=false
```

如需使用其他 URDF/Xacro 或 RViz2 配置：

```bash
ros2 launch ranger_mini_v3 display.launch.py \
  model:=/absolute/path/to/model.xacro \
  rviz_config:=/absolute/path/to/config.rviz
```

### Gazebo Classic 运动仿真

启动 Gazebo、生成模型并加载 ros2_control 控制器：

```bash
ros2 launch ranger_mini_v3 gazebo.launch.py
```

等待终端显示三个控制器均为 `active` 后，在另一个已加载工作区环境的终端中
启动键盘控制：

```bash
source /opt/ros/humble/setup.bash
source ~/ranger_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`teleop_twist_keyboard` 默认向 `/cmd_vel` 发布 `geometry_msgs/msg/Twist`。由 AgileX
官方 Humble 仿真包迁入的 `four_ws_control` 会在同相转向、反相转向和原地旋转
三种模式之间自动切换，并分别计算四个转向角和四个车轮角速度。按空格键可停车。

无界面运行：

```bash
ros2 launch ranger_mini_v3 gazebo.launch.py gui:=false
```

指定初始位置和朝向：

```bash
ros2 launch ranger_mini_v3 gazebo.launch.py \
  x:=1.0 y:=0.5 z:=0.0 yaw:=1.57
```

指定自己的 world：

```bash
ros2 launch ranger_mini_v3 gazebo.launch.py \
  world:=/absolute/path/to/world.sdf
```

可用参数包括：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `model` | 包内 `ranger_mini.xacro` | URDF 或 Xacro 的绝对路径 |
| `entity_name` | `ranger_mini_v3` | Gazebo 实体名称 |
| `world` | Gazebo 空世界 | world 文件路径 |
| `gui` | `true` | 是否启动 Gazebo 客户端 |
| `paused` | `false` | 是否暂停启动 |
| `x`, `y`, `z` | `0.0` | 初始位置，单位为米 |
| `yaw` | `0.0` | 初始偏航角，单位为弧度 |

检查 ros2_control 状态：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

正常情况下，`joint_state_broadcaster`、`forward_position_controller` 和
`forward_velocity_controller` 均应显示为 `active`，八个 command interface 均应为
`[claimed]`。

## ROS 接口

### 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `port_name` | `can0` | CAN 接口名称 |
| `robot_model` | `ranger` | `ranger`、`ranger_mini_v1`、`ranger_mini_v2` 或 `ranger_mini_v3` |
| `update_rate` | `50` | 底盘状态更新频率，单位为 Hz |
| `base_frame` | `base_link` | 机器人基座坐标系 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `publish_odom_tf` | `false` | 是否发布 `odom -> base_link` TF |
| `odom_topic_name` | `odom` | 里程计话题名称 |

### 发布的话题

- `/system_state`（`ranger_msgs/msg/SystemState`）
- `/motion_state`（`ranger_msgs/msg/MotionState`）
- `/actuator_state`（`ranger_msgs/msg/ActuatorStateArray`）
- `/odom`（`nav_msgs/msg/Odometry`）
- `/battery_state`（`sensor_msgs/msg/BatteryState`）

### 订阅的话题

- `/cmd_vel`（`geometry_msgs/msg/Twist`）

## 上游仓库

- Ranger ROS 2 驱动：<https://github.com/agilexrobotics/ranger_ros2>
- UGV SDK：<https://github.com/agilexrobotics/ugv_sdk>
- Ranger Mini V3 官方模型：
  <https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/ranger_mini/ranger_mini_v3>
- Ranger ROS 2 Humble 仿真与四轮转向控制：
  <https://github.com/agilexrobotics/ugv_gazebo_sim/tree/humble/ranger>

## License

本仓库沿用上游项目的 BSD 3-Clause License，详见 [LICENSE](./LICENSE)。

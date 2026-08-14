# TurtleBot3 + FR3 导航仿真

TurtleBot3 Waffle Pi + FR3 机械臂在 **Gazebo Fortress (Ignition Gazebo)** 中的仿真包，
包含机器人仿真、仿真场景、导航（Nav2）与串行巡逻任务节点。

## 包含内容

| 包 / 目录 | 说明 |
|---|---|
| `franka_description` | FR3 机械臂 URDF/meshes（tag 2.8.1），仿真 URDF 的硬依赖 |
| `turtlebot3_manipulation_gazebo` | 仿真 spawn 启动（`turtlebot3_franka.launch.py`）、TB3+FR3 URDF、ros2_control 配置、网格 |
| `turtlebot3_manipulation_navigation2` | Nav2 启动（`navigation.launch.py`）、地图/参数、任务节点 `patrol_task.py`、cmd_vel 桥接 |
| `wpr_simulation_ros2` | 仿真场景资源（`worlds/example.world` + `models/`） |
| `docs/` | 两份说明文档（见下） |

## 目录结构

```
.
├── franka_description/                 # FR3 机械臂 URDF/meshes（依赖）
├── turtlebot3_manipulation_gazebo/     # 仿真包
├── turtlebot3_manipulation_navigation2/  # 导航 + 任务包
├── wpr_simulation_ros2/                # 仿真场景包
├── docs/
│   ├── turtlebot3-fr3-fortress-integration.md  # TB3+FR3 集成细节（含改动清单）
│   └── task-navigation.md              # 任务节点关键配置（位姿/顺序/流程）
└── README.md
```

## 环境要求

- **ROS 2 Humble**
- **Gazebo Fortress (Ignition Gazebo)** + `ros_gz_sim` / `ros_gz_bridge` / `gz_ros2_control`
- **Navigation2**：`ros-humble-navigation2`、`ros-humble-nav2-bringup`

## 额外依赖（apt 安装即可，本仓库已含源码包）

| 依赖 | 安装 |
|---|---|
| `xacro`、`robot_state_publisher`、`joint_state_publisher_gui` | `sudo apt install ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui` |
| `ros2_control`、`ros2_controllers`、`gripper_controllers` | `sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gripper-controllers` |
| `ros_gz_sim`、`ros_gz_bridge`、`gz_ros2_control` | `sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-gz-ros2-control` |
| `rviz2` | `sudo apt install ros-humble-rviz2` |

> `franka_description` 已打包在本仓库里，无需再单独 clone。

## 构建

把本仓库四个包放到你自己的 `turtlebot3_ws/src/` 下（复制或软链均可），然后：

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install \
  --packages-select franka_description \
  turtlebot3_manipulation_gazebo \
  turtlebot3_manipulation_navigation2 \
  wpr_simulation_ros2
source install/setup.bash
```

> 用 `--symlink-install`，之后改 `patrol_task.py` 等 Python 脚本无需重新编译。

## 运行（三个终端）

**终端 1 — 仿真 + spawn 机器人（含 FR3 臂）**

```bash
ros2 launch turtlebot3_manipulation_gazebo turtlebot3_franka.launch.py
```

**终端 2 — 导航（Nav2 + 地图 + 参数）**

```bash
ros2 launch turtlebot3_manipulation_navigation2 navigation.launch.py
```

**终端 3 — 任务节点（初始定位 → 串行巡逻）**

```bash
ros2 run turtlebot3_manipulation_navigation2 patrol_task.py
```

任务节点会依次导航到 4 个客厅观察点（顺序 `table_3 → table_1 → table_0 → table_2`），
到达后站稳扫描，再移动到下一个。详见 `docs/task-navigation.md`。

## 文档说明

- `docs/turtlebot3-fr3-fortress-integration.md` — TB3+FR3 在 Fortress 的集成改动清单、控制器/传感器验证结果、转运姿态关节值。
- `docs/task-navigation.md` — 任务节点的初始位姿、访问顺序、串行导航流程、地图坐标系约定。

# `wpr_simulation_ros2`

这是面向 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Fortress 6 的
仿真场景资源包。它是独立项目，不依赖原 ROS 1 `wpr_simulation` 包。

包内提供：

- `worlds/example.world`
- `worlds/official.world`
- `models/`：场景引用的模型、网格与纹理
- `launch/world.launch.py`

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Fortress 6
- `ros_gz`

检查依赖：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
ign gazebo --versions
```

`ign gazebo --versions` 应包含版本 `6`。

## 构建

将本项目作为 ROS 2 包放入 colcon 工作空间：

```bash
mkdir -p ~/wpr_ros2_ws/src
cp -a wpr_simulation_ros2 ~/wpr_ros2_ws/src/
cd ~/wpr_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```



## 启动

启动示例场景：

```bash
ros2 launch wpr_simulation_ros2 world.launch.py world_type:=example
```

启动正式场景：

```bash
ros2 launch wpr_simulation_ros2 world.launch.py world_type:=official
```

常用参数：

```bash
ros2 launch wpr_simulation_ros2 world.launch.py gui:=false
ros2 launch wpr_simulation_ros2 world.launch.py paused:=true
ros2 launch wpr_simulation_ros2 world.launch.py verbose:=true
ros2 launch wpr_simulation_ros2 world.launch.py bridge_clock:=false
```

参数说明：

- `world_type`：只能是 `example` 或 `official`
- `gui`：是否启动 Gazebo 图形界面
- `paused`：是否以暂停状态启动
- `verbose`：是否输出 Gazebo 调试日志
- `bridge_clock`：是否将 Gazebo 时间桥接到 ROS 2 `/clock`，默认开启



## 资源解析

`package.xml` 将安装后的 `models/` 导出为 Gazebo 模型路径。通过
`ros_gz_sim` 启动时，包内模型目录会自动加入
`IGN_GAZEBO_RESOURCE_PATH`，无需再将模型复制到 `~/.gazebo/models`。

直接使用 `ign gazebo` 启动 world 时，需要先设置模型路径：

```bash
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH="$(ros2 pkg prefix --share wpr_simulation_ros2)/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
ign gazebo -r "$(ros2 pkg prefix --share wpr_simulation_ros2)/worlds/example.world"
```

推荐始终使用 `ros2 launch`，它同时处理资源路径、Gazebo 版本和 `/clock`  
桥接。



## Fortress 6.16 图形界面警告

Fortress 6.16 的默认 GUI 在 Ubuntu 22.04 上可能输出 QML deprecated、
`Unable to deserialize sdf::Model` 等警告。相同警告也可能出现在 Fortress
自带的 `shapes.sdf` 场景中；只要模型完整显示且服务端持续运行，它们不是
本场景资源缺失的提示。

若出现以下警告：

```text
libEGL warning: egl: failed to create dri2 screen
```

它表示宿主机 OpenGL/EGL 驱动不可用或未正确加载，而不是 world 文件错误。
应先检查显卡驱动，例如：

```bash
nvidia-smi
glxinfo -B
```

无图形界面运行可避开 GUI 渲染路径：

```bash
ros2 launch wpr_simulation_ros2 world.launch.py world_type:=example gui:=false
```


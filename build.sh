#!/usr/bin/env bash
# 干净环境构建（可选，推荐用）。
#
# 为什么需要：colcon build 会把「构建那一刻终端里已有的 ROS 前缀」原样写进
# install/setup.bash 的链里。如果在脏终端（source 过其它 ROS 安装 / 本地
# ROS 副本）里构建，脏前缀会被永久记进去，之后每次 source 都复活，导致
# nav2 / ros_gz 混搭、导航被拒。本脚本先清空所有 ROS 前缀，只 source 系统 ROS。
#
# 用法（在仓库根目录，仓库已放到 <ws>/src/turtlebot3_franka/ 下）:
#   bash build.sh
#
# 之后记得 source:
#   source <ws>/install/setup.bash

set -eo pipefail

# ── 1. 清空所有 ROS 相关前缀，阻断脏终端环境继承 ──────────────
unset AMENT_PREFIX_PATH \
      COLCON_PREFIX_PATH \
      CMAKE_PREFIX_PATH \
      ROS_PACKAGE_PATH \
      2>/dev/null || true

# ── 2. 只 source 系统 ROS（唯一允许的底层环境）────────────────
source /opt/ros/humble/setup.bash

# ── 3. 定位工作区根目录（本仓库在 <ws>/src/turtlebot3_franka/）──
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS_ROOT"

# ── 4. 构建 ────────────────────────────────────────────────────
colcon build --symlink-install \
  --packages-select franka_description \
                   turtlebot3_manipulation_gazebo \
                   turtlebot3_manipulation_navigation2 \
                   wpr_simulation_ros2

echo ""
echo "✅ 构建完成（环境干净）。"
echo "   接下来执行:  source $WS_ROOT/install/setup.bash"

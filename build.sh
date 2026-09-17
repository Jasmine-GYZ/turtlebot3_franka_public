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

# ── 3.5 修 setuptools / packaging 版本冲突（rosidl 生成 Python 类型时才会撞上）──
# 现象（只在带 msg/srv 的包上出现，如 turtlebot3_manipulation_grasp）：
#   TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'
# 原因：ament_cmake_python 会调用 ~/.local 的 setuptools 83.0.0，而它需要
#   packaging >= 22；本机只有 apt 的 21.3（ros2cli/colcon/rosdistro 都依赖它，不能换）。
# 为什么不干脆屏蔽用户级包（PYTHONNOUSERSITE=1）：
#   rosidl 生成必需的 empy 只在 ~/.local 里装，屏蔽掉会直接失败 ✗
# 解法：往工作区内的隔离目录装一份新版 packaging，仅通过 PYTHONPATH 注入本次构建。
#   不碰 ~/.local、不碰系统；想回退删掉 $WS_ROOT/.build_pydeps 即可。
PYDEPS="$WS_ROOT/.build_pydeps"
if [ ! -d "$PYDEPS/packaging" ]; then
  echo "首次构建：往 $PYDEPS 装一份隔离的 packaging（不污染系统环境）..."
  pip install --target="$PYDEPS" --no-deps --quiet packaging
fi
export PYTHONPATH="$PYDEPS${PYTHONPATH:+:$PYTHONPATH}"

# ── 4. 构建 ────────────────────────────────────────────────────
colcon build --symlink-install \
  --packages-select franka_description \
                   turtlebot3_manipulation_gazebo \
                   turtlebot3_manipulation_navigation2 \
                   turtlebot3_manipulation_grasp \
                   turtlebot3_moveit_config \
                   wpr_simulation_ros2

echo ""
echo "✅ 构建完成（环境干净）。"
echo "   接下来执行:  source $WS_ROOT/install/setup.bash"

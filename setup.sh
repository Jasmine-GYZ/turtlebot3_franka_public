#!/usr/bin/env bash
# 一键搭建 turtlebot3_franka 的完整依赖环境。
#
# 完成以下步骤:
#   1. 校验 ROS 2 Humble 并 source
#   2. git submodule 拉取第三方源码（sam2 / INSID3 / dinov3）
#   3. apt 安装 ROS 包
#   4. 创建 Python 虚拟环境并安装视觉识别依赖（torch + requirements.txt）
#   5. 下载模型权重（DINOv3 门控权重需手动获取，见 download_weights.sh）
#
# 用法（在仓库根目录）:
#   bash setup.sh                 # CPU 版 torch（无 GPU 也能跑，仅测试）
#   bash setup.sh --cuda          # CUDA 版 torch（有 NVIDIA GPU 时用）
#   bash setup.sh --skip-apt      # 跳过 apt 安装（已装过 ROS 包时用）
#   bash setup.sh --skip-weights  # 跳过权重下载
#
# 可用环境变量:
#   VISION_ENV_DIR      虚拟环境目录（默认 ~/vision_env）
#   MODEL_WEIGHTS_DIR   权重根目录（默认 ~/model_weights）

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VISION_ENV_DIR="${VISION_ENV_DIR:-$HOME/vision_env}"
export MODEL_WEIGHTS_DIR="${MODEL_WEIGHTS_DIR:-$HOME/model_weights}"

USE_CUDA=0
SKIP_APT=0
SKIP_WEIGHTS=0
for arg in "$@"; do
  case "$arg" in
    --cuda) USE_CUDA=1 ;;
    --skip-apt) SKIP_APT=1 ;;
    --skip-weights) SKIP_WEIGHTS=1 ;;
    *) echo "未知参数: $arg"; exit 1 ;;
  esac
done

echo "════════════════════════════════════════════════"
echo "  turtlebot3_franka 环境搭建"
echo "  仓库:      $REPO_ROOT"
echo "  虚拟环境:  $VISION_ENV_DIR"
echo "  权重目录:  $MODEL_WEIGHTS_DIR"
echo "════════════════════════════════════════════════"

# ── 1. ROS 2 Humble ────────────────────────────────────────────
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "❌ 未找到 /opt/ros/humble/setup.bash，请先安装 ROS 2 Humble（Ubuntu 22.04）。"
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
echo "✔ ROS 2 Humble: $ROS_DISTRO"

# ── 2. 第三方源码 submodule ────────────────────────────────────
cd "$REPO_ROOT"
if [ -d .git ]; then
  git submodule update --init --recursive
  echo "✔ 第三方 submodule（sam2 / INSID3 / dinov3）已就绪"
else
  echo "⚠ 当前目录不是 git 仓库，跳过 submodule（请用 git clone --recursive 获取）。"
fi

# ── 3. apt 安装 ROS 包 ─────────────────────────────────────────
if [ "$SKIP_APT" -eq 0 ]; then
  echo "── 安装 ROS 包（需 sudo，可能提示输入密码）──"
  sudo apt-get update
  sudo apt-get install -y \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gripper-controllers \
    ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-gz-ros2-control \
    ros-humble-rviz2 \
    python3-venv python3-pip
  echo "✔ ROS 包安装完成"
else
  echo "⊘ 跳过 apt 安装（--skip-apt）"
fi

# ── 4. Python 虚拟环境 ─────────────────────────────────────────
echo "── 创建虚拟环境 $VISION_ENV_DIR ──"
python3 -m venv "$VISION_ENV_DIR"
# shellcheck disable=SC1091
source "$VISION_ENV_DIR/bin/activate"
python -m pip install --upgrade pip

if [ "$USE_CUDA" -eq 1 ]; then
  echo "── 安装 CUDA 版 PyTorch ──"
  pip install torch torchvision torchaudio
else
  echo "── 安装 CPU 版 PyTorch ──"
  pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
fi

echo "── 安装视觉识别依赖（requirements.txt）──"
pip install -r "$REPO_ROOT/requirements.txt"
echo "✔ 虚拟环境就绪: $VISION_ENV_DIR"

# ── 5. 模型权重 ────────────────────────────────────────────────
if [ "$SKIP_WEIGHTS" -eq 0 ]; then
  bash "$REPO_ROOT/download_weights.sh"
else
  echo "⊘ 跳过权重下载（--skip-weights）"
fi

echo ""
echo "════════════════════════════════════════════════"
echo "  ✅ 环境搭建完成"
echo ""
echo "  后续步骤:"
echo "    1) 把本仓库放到你的工作区，例如 ~/turtlebot3_ws/src/turtlebot3_franka"
echo "    2) 构建（在 ~/turtlebot3_ws 下）:"
echo "         bash ~/turtlebot3_ws/src/turtlebot3_franka/../build.sh   # 或按 README 用 colcon"
echo "    3) 运行前先激活虚拟环境:"
echo "         source $VISION_ENV_DIR/bin/activate"
echo "    4) 三个终端分别启动（见 README）"
echo "════════════════════════════════════════════════"

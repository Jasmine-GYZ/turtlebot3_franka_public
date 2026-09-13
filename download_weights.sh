#!/usr/bin/env bash
# 下载视觉识别所需的 3 个模型权重。
#
# 两个公开权重（GroundingDINO / SAM2）本脚本会自动下载到 $MODEL_WEIGHTS_DIR；
# DINOv3 骨干权重是官方门控下载，脚本只打印获取方式，需手动放入指定目录。
#
# 用法（在仓库根目录，或任意目录均可）:
#   bash download_weights.sh
#
# 可用环境变量:
#   MODEL_WEIGHTS_DIR   权重根目录（默认 ~/model_weights）

set -euo pipefail

# 仓库根目录（本脚本所在目录）
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODEL_WEIGHTS_DIR="${MODEL_WEIGHTS_DIR:-$HOME/model_weights}"
GDINO_DIR="$MODEL_WEIGHTS_DIR/groundingdino"
SAM2_DIR="$MODEL_WEIGHTS_DIR/sam2"
CKPT_DIR="$REPO_ROOT/turtlebot3_manipulation_navigation2/scripts/checkpoints"

mkdir -p "$GDINO_DIR" "$SAM2_DIR" "$CKPT_DIR"

# ── 1. GroundingDINO Swin-T OGC ───────────────────────────────
GDINO_URL="https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth"
GDINO_PATH="$GDINO_DIR/groundingdino_swint_ogc.pth"

if [ -s "$GDINO_PATH" ]; then
  echo "✔ GroundingDINO 权重已存在: $GDINO_PATH"
else
  echo "下载 GroundingDINO Swin-T OGC (~662MB)..."
  wget -q --show-progress -O "$GDINO_PATH" "$GDINO_URL" || curl -L --progress-bar -o "$GDINO_PATH" "$GDINO_URL"
fi

# ── 2. SAM2 Hiera-Small ───────────────────────────────────────
SAM2_URL="https://dl.fbaipublicfiles.com/segment_anything_2/072824/sam2_hiera_small.pt"
SAM2_PATH="$SAM2_DIR/sam2_hiera_small.pt"

if [ -s "$SAM2_PATH" ]; then
  echo "✔ SAM2 权重已存在: $SAM2_PATH"
else
  echo "下载 SAM2 Hiera-Small (~176MB)..."
  wget -q --show-progress -O "$SAM2_PATH" "$SAM2_URL" || curl -L --progress-bar -o "$SAM2_PATH" "$SAM2_URL"
fi

# ── 3. DINOv3 Base 骨干（官方门控，无法自动下载）──────────────
DINOV3_FILENAME="dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth"
DINOV3_PATH="$CKPT_DIR/$DINOV3_FILENAME"

if [ -s "$DINOV3_PATH" ]; then
  echo "✔ DINOv3 骨干权重已存在: $DINOV3_PATH"
else
  cat <<EOF

⚠ DINOv3 骨干权重需手动下载（Meta 官方门控，无法自动获取）:
   文件名: $DINOV3_FILENAME  (~342MB)
   获取方式（二选一）:
     1) https://ai.meta.com/resources/models-and-libraries/dinov3-downloads/ 申请后邮件给 URL；
     2) HuggingFace 仓库 facebook/dinov3-vitb16-pretrain-lvd1689m（需授权）取原生 .pth。
   下载后放入:
     $DINOV3_PATH
   （缺失时视觉流水线会自动降级为「无 INSID3 复核」，不影响 GroundingDINO+SAM2 检测）
EOF
fi

echo ""
echo "完成。权重目录: $MODEL_WEIGHTS_DIR"
echo "DINOv3 目录:    $CKPT_DIR"

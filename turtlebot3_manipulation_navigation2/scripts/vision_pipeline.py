#!/usr/bin/env python3
"""
视觉识别流水线 — GroundingDINO（开放集检测）+ SAM2（框提示实例分割）

对应离线脚本 sam2/run_pipeline.py 的在线化封装，供 patrol_task.py 复用：

    图像(BGR) → GroundingDINO 检测 → NMS + 置信度过滤
              → 框转像素 xyxy → SAM2 实例分割
              → 计算每个实例的掩码 + 像素中心点 → 返回结果列表

依赖环境：/home/jasmine/vision_env（同时含 torch/groundingdino/sam2 与 rclpy）。
运行本文件可离线自测：  python3 vision_pipeline.py <图片路径>
"""

import os
import sys

# GroundingDINO 的 bert tokenizer 默认会去 HuggingFace 在线下载；本机离线或
# 证书校验失败会导致下载失败、模型加载中断。这里强制走本地缓存
# （bert-base-uncased tokenizer 已缓存在 ~/.cache/huggingface），
# 与离线脚本 run_pipeline.py 顶部的 HF_HUB_OFFLINE=1 保持一致。
os.environ.setdefault("HF_HUB_OFFLINE", "1")
os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")

# ─────────────────────────────────────────────────────────────
# 修复 `import sam2` 被旧仓库目录 shadow 的问题
# ─────────────────────────────────────────────────────────────
# /home/jasmine 下还有一个旧副本 /home/jasmine/sam2（无 __init__.py 的
# 命名空间包），会抢在真正的 sam2 包之前被导入，触发官方 build_sam.py 的
# RuntimeError。这里把 src/sam2 仓库根目录插到 sys.path 最前面，
# 确保 `import sam2` 解析到正确的包（sam2/sam2/）。
_GDINO_REPO = "/home/jasmine/turtlebot3_ws/src/GroundingDINO"   # 仅用于取 config 路径
_SAM2_REPO = "/home/jasmine/turtlebot3_ws/src/sam2"

# 注意：不要连 _GDINO_REPO 也塞进 sys.path —— 仓库里那版 groundingdino 是
# v0.1.0，其 ms_deform_attn.py 缺 key_padding_mask 参数，与 transformer.py
# 不匹配，推理会报 TypeError。vision_env 里已装 v0.3.0（可用），直接用已安装版。
if _SAM2_REPO not in sys.path:
    sys.path.insert(0, _SAM2_REPO)

import cv2
import numpy as np


# ═══════════════════════════════════════════════════════════════
# 配置区（路径与阈值，与 sam2/run_pipeline.py 保持一致）
# ═══════════════════════════════════════════════════════════════

GDINO_CONFIG = os.path.join(_GDINO_REPO, "groundingdino", "config",
                            "GroundingDINO_SwinT_OGC.py")
GDINO_CKPT = "/home/jasmine/model_weights/groundingdino/groundingdino_swint_ogc.pth"

SAM2_CKPT = "/home/jasmine/model_weights/sam2/sam2_hiera_small.pt"
SAM2_CONFIG_NAME = "sam2_hiera_s.yaml"

# 待计数物品（英文名）。GroundingDINO 的 caption 用 " . " 分隔多个文本查询，
# 每个框返回的 phrase 就是命中的那一个查询文本。
ITEM_NAMES = ["apple", "coke can", "bowl", "banana"]
# 每种物品的别名（用于把检测到的 phrase 归一到待计数物品之一）。
ITEM_ALIASES = {
    "apple": ["apple", "red round apple"],
    "coke can": ["coke", "can", "coca", "cola", "soda"],
    "bowl": ["bowl"],
    "banana": ["banana"],
}
TEXT_PROMPT = " . ".join(ITEM_NAMES) + " ."

BOX_THRESHOLD = 0.30
TEXT_THRESHOLD = 0.30
NMS_IOU_THRESH = 0.55
# 最终保留阈值：不能太高——table_3 的真苹果在不同运行里只有 0.43~0.58，提到 0.50
# 会把它误杀。空桌假阳性（桌腿被认成 coke）不用分数过滤，改由 patrol_task 里的
# 高度过滤（/map z 低于桌面高度即丢弃）处理。
CONF_KEEP = 0.38


def classify_phrase(phrase):
    """把 GroundingDINO 返回的 phrase 归一到 ITEM_NAMES 之一，失败返回 None。"""
    p = (phrase or "").strip().lower()
    for name in ITEM_NAMES:
        aliases = ITEM_ALIASES.get(name, [name])
        if any(a in p for a in aliases):
            return name
    return None


class VisionPipeline:
    """GroundingDINO + SAM2 一体化检测/分割器。

    重型依赖（torch / groundingdino / sam2）延迟到 __init__ 里再导入，
    避免 patrol_task.py 在节点启动阶段就被拖慢。
    """

    def __init__(self, device=None, text_prompt=None):
        import torch

        # 重型导入延迟到此处
        from torchvision.ops import nms, box_convert
        from groundingdino.util.inference import load_model, predict, Model
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.text_prompt = text_prompt or TEXT_PROMPT
        self._nms = nms
        self._box_convert = box_convert
        self._gdino_predict = predict
        self._preprocess = Model.preprocess_image

        print("===== 加载 SAM2（{}）=====".format(SAM2_CONFIG_NAME), flush=True)
        self.sam_model = build_sam2(SAM2_CONFIG_NAME, SAM2_CKPT, device=self.device)
        self.sam_predictor = SAM2ImagePredictor(self.sam_model)
        print("✅ SAM2 加载完成", flush=True)

        print("===== 加载 GroundingDINO =====", flush=True)
        self.gdino_model = load_model(GDINO_CONFIG, GDINO_CKPT, device=self.device)
        print("✅ GroundingDINO 加载完成", flush=True)

    # ── 主流程 ───────────────────────────────────────────────

    def detect(self, img_bgr):
        """对一帧 BGR 图像做检测 + 分割。

        返回 list[dict]，每个元素：
            box_xyxy : np.ndarray [x1, y1, x2, y2]（像素，float）
            score    : float 置信度
            phrase   : str  类别文本
            mask     : np.ndarray (H, W) bool 实例掩码
            center_px: (cx, cy) 掩码质心（像素）
            area     : int  掩码面积（像素数）
        """
        image_transformed = self._preprocess(img_bgr).to(self.device)

        boxes, logits, phrases = self._gdino_predict(
            model=self.gdino_model,
            image=image_transformed,
            caption=self.text_prompt,
            box_threshold=BOX_THRESHOLD,
            text_threshold=TEXT_THRESHOLD,
            device=self.device,
        )

        if boxes.shape[0] == 0:
            return []

        # GroundingDINO 输出是归一化 cxcywh，先转成像素 xyxy（NMS 与 SAM2 都需要）
        h, w = img_bgr.shape[:2]
        boxes_xyxy = self._box_convert(
            boxes * boxes.new_tensor([w, h, w, h]),
            in_fmt="cxcywh", out_fmt="xyxy",
        )

        # NMS 去重叠（在像素 xyxy 上算 IoU 才正确）
        keep = self._nms(boxes_xyxy, logits, iou_threshold=NMS_IOU_THRESH)
        boxes_xyxy = boxes_xyxy[keep]
        logits = logits[keep]
        phrases = [phrases[i] for i in keep]

        # 置信度阈值过滤
        conf_mask = logits > CONF_KEEP
        boxes_xyxy = boxes_xyxy[conf_mask]
        logits = logits[conf_mask]
        phrases = [p for p, m in zip(phrases, conf_mask.tolist()) if m]

        if boxes_xyxy.shape[0] == 0:
            return []

        # 中心去重：GroundingDINO 可能把同一物体用不同 phrase 各框一次
        # （如同一苹果既命中 "apple" 又命中 "coke"），框中心几乎重合时只留得分最高者。
        dedup_idx = self._dedup_by_center(boxes_xyxy)
        boxes_xyxy = boxes_xyxy[dedup_idx]
        logits = logits[dedup_idx]
        phrases = [phrases[i] for i in dedup_idx]

        # SAM2：框提示实例分割（期望绝对像素 xyxy）
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        self.sam_predictor.set_image(img_rgb)
        masks, _, _ = self.sam_predictor.predict(
            box=boxes_xyxy.cpu().numpy().astype(np.float32),
            multimask_output=False,
        )

        detections = []
        for box_xyxy, score, phrase, mask in zip(boxes_xyxy, logits, phrases, masks):
            mask_bin = np.squeeze(mask) > 0.5  # (H, W) bool
            center_px = self._mask_center(mask_bin)
            detections.append({
                "box_xyxy": box_xyxy.cpu().numpy().astype(np.float32),
                "score": float(score),
                "phrase": phrase,
                "mask": mask_bin,
                "center_px": center_px,
                "area": int(mask_bin.sum()),
            })

        return detections

    @staticmethod
    def _mask_center(mask_bin):
        """掩码质心（像素中心点）。掩码为空时返回 (0, 0)。"""
        ys, xs = np.nonzero(mask_bin)
        if xs.size == 0:
            return (0.0, 0.0)
        return (float(xs.mean()), float(ys.mean()))

    @staticmethod
    def _dedup_by_center(boxes_xyxy, dist_px=30.0):
        """按框中心距离合并重复框（同一物体被不同 phrase 框多次）。

        boxes_xyxy 已按得分从高到低排列（NMS 输出），因此先出现的高分框
        获胜；后续框中心落在 dist_px 内视为同一物体，丢弃。返回保留的索引列表。
        """
        boxes = boxes_xyxy.detach().cpu().numpy()
        centers = (boxes[:, :2] + boxes[:, 2:]) / 2.0  # (cx, cy)
        keep = []
        for i in range(len(boxes)):
            cx, cy = centers[i]
            dup = False
            for j in keep:
                kx, ky = centers[j]
                if (cx - kx) ** 2 + (cy - ky) ** 2 <= dist_px * dist_px:
                    dup = True
                    break
            if not dup:
                keep.append(i)
        return keep

    # ── 可视化 ───────────────────────────────────────────────

    def annotate(self, img_bgr, detections):
        """在图像上绘制检测框 + 分割轮廓 + 中心点，返回 BGR 图。"""
        img = img_bgr.copy()
        for d in detections:
            x1, y1, x2, y2 = [int(v) for v in d["box_xyxy"]]
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = "{} {:.2f}".format(d["phrase"], d["score"])
            cv2.putText(img, label, (x1, max(y1 - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            mask_u8 = d["mask"].astype(np.uint8) * 255
            contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, contours, -1, (0, 255, 255), 2)

            cx, cy = d["center_px"]
            cv2.drawMarker(img, (int(cx), int(cy)), (0, 0, 255),
                           cv2.MARKER_CROSS, 16, 2)
        return img

    def save_annotated(self, path, img_bgr, detections):
        """保存标注结果图（自动创建父目录）。"""
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        cv2.imwrite(path, self.annotate(img_bgr, detections))


# ── 离线自测入口 ─────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("用法: python3 vision_pipeline.py <图片路径>")
        sys.exit(1)

    img_path = sys.argv[1]
    out_path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/vision_result.jpg"

    img_bgr = cv2.imread(img_path)
    if img_bgr is None:
        print("无法读取图片: {}".format(img_path))
        sys.exit(1)

    pipeline = VisionPipeline()
    detections = pipeline.detect(img_bgr)
    print("检测到 {} 个目标".format(len(detections)))
    for d in detections:
        print("  {} score={:.3f} center={}".format(
            d["phrase"], d["score"], d["center_px"]))

    pipeline.save_annotated(out_path, img_bgr, detections)
    print("结果已保存: {}".format(out_path))


if __name__ == "__main__":
    main()

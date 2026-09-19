#!/usr/bin/env python3
"""
视觉识别流水线 — GroundingDINO（开放集检测）+ SAM2（框提示实例分割）

对应离线脚本 sam2/run_pipeline.py 的在线化封装，供 patrol_task.py 复用：

    图像(BGR) → GroundingDINO 检测 → NMS + 置信度过滤
              → 框转像素 xyxy → SAM2 实例分割
              → 计算每个实例的掩码 + 像素中心点 → 返回结果列表

依赖环境：需先激活 Python 虚拟环境（含 torch/groundingdino-py/sam2 与 rclpy），
见仓库根 setup.sh / requirements.txt。
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
# 路径解析（仓库根目录 + 第三方 submodule + 模型权重）
# ─────────────────────────────────────────────────────────────
# 本文件位于 <repo>/turtlebot3_manipulation_navigation2/scripts/，向上 2 级即
# 仓库根目录，third_party 子模块与 setup.sh / requirements.txt 都在这里。
_REPO_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", ".."))

# 把 third_party/sam2 仓库根目录插到 sys.path 最前面，确保 `import sam2`
# 解析到正确的包（sam2/sam2/），避免被机器上其它同名 sam2 副本 shadow。
_SAM2_REPO = os.path.join(_REPO_ROOT, "third_party", "sam2")
if _SAM2_REPO not in sys.path:
    sys.path.insert(0, _SAM2_REPO)

# GroundingDINO 的模型代码来自已安装的 groundingdino-py（PyPI 包），其 config
# 文件（GroundingDINO_SwinT_OGC.py）随包一起安装，用 find_spec 定位即可，
# 无需额外的 GroundingDINO 仓库。注意用 find_spec 而非直接 import，避免触发
# 重型依赖的加载。
import importlib.util
_GDINO_SPEC = importlib.util.find_spec("groundingdino")
_GDINO_PKG_DIR = os.path.dirname(_GDINO_SPEC.origin) if _GDINO_SPEC else ""

# 模型权重根目录（GroundingDINO / SAM2），可用环境变量 MODEL_WEIGHTS_DIR 覆盖。
MODEL_WEIGHTS_DIR = os.environ.get("MODEL_WEIGHTS_DIR",
                                   os.path.expanduser("~/model_weights"))

import cv2
import numpy as np


# ═══════════════════════════════════════════════════════════════
# 配置区（路径与阈值，与 sam2/run_pipeline.py 保持一致）
# ═══════════════════════════════════════════════════════════════

GDINO_CONFIG = os.path.join(_GDINO_PKG_DIR, "config", "GroundingDINO_SwinT_OGC.py")
GDINO_CKPT = os.path.join(MODEL_WEIGHTS_DIR, "groundingdino", "groundingdino_swint_ogc.pth")

SAM2_CKPT = os.path.join(MODEL_WEIGHTS_DIR, "sam2", "sam2_hiera_small.pt")
SAM2_CONFIG_NAME = "sam2_hiera_s.yaml"

# 待计数物品（英文名）。GroundingDINO 的 caption 用 " . " 分隔多个文本查询，
# 每个框返回的 phrase 就是命中的那一个查询文本。
ITEM_NAMES = ["apple", "coke can", "bleach cleanser"]
# 每种物品的别名（用于把检测到的 phrase 归一到待计数物品之一）。
ITEM_ALIASES = {
    "apple": ["apple", "red round apple"],
    "coke can": ["coke", "can", "coca", "cola", "soda"],
    "bleach cleanser": ["bleach", "cleanser"],
}
TEXT_PROMPT = " . ".join(ITEM_NAMES) + " ."

BOX_THRESHOLD = 0.30
TEXT_THRESHOLD = 0.30
NMS_IOU_THRESH = 0.55
# 最终保留阈值：不能太高——table_3 的真苹果在不同运行里只有 0.43~0.58，提到 0.50
# 会把它误杀。空桌假阳性（桌腿被认成 coke）不用分数过滤，改由 patrol_task 里的
# 高度过滤（/map z 低于桌面高度即丢弃）处理。
CONF_KEEP = 0.30


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
        from groundingdino.util.inference import load_model, Model
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.text_prompt = text_prompt or TEXT_PROMPT
        self._nms = nms
        self._box_convert = box_convert
        self._preprocess = Model.preprocess_image

        print("===== 加载 SAM2（{}）=====".format(SAM2_CONFIG_NAME), flush=True)
        self.sam_model = build_sam2(SAM2_CONFIG_NAME, SAM2_CKPT, device=self.device)
        self.sam_predictor = SAM2ImagePredictor(self.sam_model)
        print("✅ SAM2 加载完成", flush=True)

        print("===== 加载 GroundingDINO =====", flush=True)
        # load_model 只加载权重不搬设备（原 predict() 内部才 .to(device)），
        # 这里改直接前向后须自己把模型搬到 device。
        self.gdino_model = load_model(GDINO_CONFIG, GDINO_CKPT, device=self.device).to(self.device)
        print("✅ GroundingDINO 加载完成", flush=True)

        # 缓存 caption 分词结果（逐框 argmax 映射短语用，与模型内部 tokenizer 一致）。
        tokenized = self.gdino_model.tokenizer(
            [self.text_prompt], padding="longest", return_tensors="pt")
        self._input_ids = tokenized["input_ids"][0].tolist()

        # ★ 诊断用：把「闭集复核前 GDINO 提了哪些框」和「复核留下了几个」留一份给调用方。
        #   复核是**静默丢弃**的（insid3_review.review 命中干扰类就 continue，一行日志都不打）
        #   ⇒ 不记这个，"某物体漏检"就分不清是 GDINO 没给框，还是给了被复核丢掉了。
        #   每次 detect() 覆盖。
        self.last_review = None

        # INSID3 闭集复核（可选）：权重缺失 / 依赖缺失时降级为「无复核」，
        # 仍能产出 GroundingDINO(argmax)+SAM2 结果，不阻塞巡逻。
        self.reviewer = None
        try:
            from insid3_review import Insid3Reviewer
            print("===== 加载 INSID3 闭集复核器（Train-Free，冻结骨干）=====", flush=True)
            self.reviewer = Insid3Reviewer(device=self.device)
            print("✅ INSID3 复核器加载完成", flush=True)
        except Exception as e:
            print("⚠️ INSID3 复核器加载失败，降级为无复核: {}".format(e), flush=True)
            self.reviewer = None

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
        import torch

        image_transformed = self._preprocess(img_bgr).to(self.device)

        # 直接前向拿原始 logits 张量，不用 predict()——predict 会把所有超过
        # text_threshold 的 token 拼成一个短语，产生 "apple coke" 这类多标签。
        with torch.no_grad():
            outputs = self.gdino_model(image_transformed[None], captions=[self.text_prompt])
        pred_logits = outputs["pred_logits"].cpu().sigmoid()[0]   # (nq, 256)
        pred_boxes = outputs["pred_boxes"].cpu()[0]               # (nq, 4) cxcywh 归一化

        if pred_boxes.shape[0] == 0:
            return []

        # 逐框对「真实文本 token」做 argmax，得到单一短语 + 该 token 的分数，
        # 从根上消除拼接标签（只在 [CLS]/[SEP]/[.]/padding 之外的 token 上取 argmax）。
        n_text = len(self._input_ids)
        box_scores, token_idx = pred_logits[:, :n_text].max(dim=1)   # (nq,)
        phrases = [self._token_segment_phrase(self._input_ids, int(k)) for k in token_idx]

        # 候选框：只按框置信度筛（等价 text_threshold=0，关闭文本过滤）。
        cand = box_scores > BOX_THRESHOLD
        boxes = pred_boxes[cand]
        box_scores = box_scores[cand]
        phrases = [p for p, m in zip(phrases, cand.tolist()) if m]

        if boxes.shape[0] == 0:
            return []

        # GroundingDINO 输出是归一化 cxcywh，先转成像素 xyxy（NMS 与 SAM2 都需要）
        h, w = img_bgr.shape[:2]
        boxes_xyxy = self._box_convert(
            boxes * boxes.new_tensor([w, h, w, h]),
            in_fmt="cxcywh", out_fmt="xyxy",
        )

        # NMS 去重叠（在像素 xyxy 上算 IoU 才正确）
        keep = self._nms(boxes_xyxy, box_scores, iou_threshold=NMS_IOU_THRESH)
        boxes_xyxy = boxes_xyxy[keep]
        logits = box_scores[keep]
        phrases = [phrases[i] for i in keep]

        # 置信度阈值过滤
        conf_mask = logits > CONF_KEEP
        boxes_xyxy = boxes_xyxy[conf_mask]
        logits = logits[conf_mask]
        phrases = [p for p, m in zip(phrases, conf_mask.tolist()) if m]

        if boxes_xyxy.shape[0] == 0:
            return []

        # INSID3 闭集复核：命中目标类 → 保留；命中干扰类 / 低置信 → 丢弃。
        # ★ 判据是「crop 的 DINOv3 特征 vs 各类原型 的 argmax」，**完全不看 GDINO 分数**
        #   ⇒ 一个 0.9 分的框也可能因为原型判成干扰类被删掉（苹果曾整批被判成罐头）。
        #   这里把复核前后的情况记进 last_review，供调用方打日志。
        self.last_review = None
        if self.reviewer is not None:
            self.last_review = {
                "proposed": [{"phrase": p, "score": round(float(s), 3)}
                             for p, s in zip(phrases, logits)],
                "kept": None,
                "kept_labels": [],
                "details": None,
            }
            keep_idx, kept_labels, details = self.reviewer.review(img_bgr, boxes_xyxy)
            # 每条明细补上 GDINO 原始 phrase/score，调用方一行日志就能看到
            # 「GDINO 说 apple，复核对 apple/tomato_soup_can 的相似度各是多少」。
            for d in details:
                bi = d["box_idx"]
                d["phrase"] = phrases[bi]
                d["gscore"] = round(float(logits[bi]), 3)
            self.last_review["kept"] = len(keep_idx)
            self.last_review["kept_labels"] = list(kept_labels)
            self.last_review["details"] = details
            if len(keep_idx) == 0:
                return []
            boxes_xyxy = boxes_xyxy[keep_idx]
            logits = logits[keep_idx]
            phrases = kept_labels

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

    def _token_segment_phrase(self, input_ids, idx):
        """把 argmax 命中的 token 下标映射回其所属短语段（特殊 token 之间的一段）。

        input_ids: caption 分词后的 token id 列表；idx: argmax 命中的下标。
        特殊 token 即各短语的分隔符：[CLS]=101、[SEP]=102、[.]=1012、[?]=1029，
        与 GroundingDINO 内部 specical_tokens 一致。
        """
        special = {101, 102, 1012, 1029}
        start = idx
        while start - 1 >= 0 and input_ids[start - 1] not in special:
            start -= 1
        end = idx
        while end + 1 < len(input_ids) and input_ids[end + 1] not in special:
            end += 1
        return self.gdino_model.tokenizer.decode(input_ids[start:end + 1]).strip()

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
        """保存标注结果图（自动创建父目录）。返回是否写盘成功。

        ★ 以前不返回、调用方也不检查 cv2.imwrite 的返回值 ⇒ 写盘失败时日志照样打
        "结果图已保存"，拿着那句话去查漏检会白跑一趟。
        """
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        return bool(cv2.imwrite(path, self.annotate(img_bgr, detections)))


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

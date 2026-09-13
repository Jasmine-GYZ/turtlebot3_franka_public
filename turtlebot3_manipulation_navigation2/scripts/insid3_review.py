#!/usr/bin/env python3
"""
INSID3 闭集复核模块（Train-Free，只推理、零训练）

对应技术文档【路线二】的「分类模型」环节：

    对 GroundingDINO 产出的每个候选框，从原图 crop 出该框的图像 patch，
    用冻结的 INSID3（DINOv3 骨干 + 位置偏置去相关）提取去偏置后的密集特征，
    与预设固定物品集合的参考原型做余弦相似度 argmax，得到该 patch 的闭集类别：

      - 命中目标类  → 保留该框，并用该类别修正 GroundingDINO 可能反转的标签；
      - 命中干扰类  → 直接丢弃该候选框；
      - 置信度过低  → 判为背景/未知，丢弃。

    骨干网络全部冻结（requires_grad=False），禁止任何微调/训练，只做推理。

依赖 / 权重存放路径（重要）：
    - INSID3 代码仓库 clone 到 src/INSID3（本模块惰性 import，避免裸
      models/utils 包名污染其它模块）。
    - DINOv3 骨干权重（约 342MB）放在本文件同级 checkpoints/ 目录：

          checkpoints/dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth

      注意：该权重官方门控（Meta 申请下载 / HuggingFace 授权），详见
      INSID3_CKPT 注释。权重缺失时本模块会抛出带说明的异常，上层
      vision_pipeline 会捕获并降级为「无复核」继续运行。
"""

import os
import sys

import cv2
import numpy as np

# ─────────────────────────────────────────────────────────────
# 复用 INSID3 / dinov3 代码（与 vision_pipeline.py 里 sam2 的写法一致）
# ─────────────────────────────────────────────────────────────
# INSID3 内部用裸 `models`/`utils` 包名，插入 sys.path 最前并只在
# 本模块内惰性 import，避免与其它包冲突。
# 本文件位于 <repo>/turtlebot3_manipulation_navigation2/scripts/，向上 2 级即
# 仓库根目录，third_party 子模块就在这里。
_REPO_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", ".."))
_INSID3_REPO = os.path.join(_REPO_ROOT, "third_party", "INSID3")
_DINOV3_REPO = os.path.join(_REPO_ROOT, "third_party", "dinov3")
for _p in (_INSID3_REPO, _DINOV3_REPO):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# ═══════════════════════════════════════════════════════════════
# 配置区（类别集合集中在这里，便于后续修改）
# ═══════════════════════════════════════════════════════════════

# DINOv3 骨干权重目录（相对本文件，与 cwd 无关）。
# 权重文件 ~342MB：dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth
# 下载方式二选一（官方门控）：
#   1) https://ai.meta.com/resources/models-and-libraries/dinov3-downloads/ 申请后邮件给 URL；
#   2) HuggingFace `facebook/dinov3-vitb16-pretrain-lvd1689m`（需授权）取原生 .pth。
INSID3_CKPT_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "checkpoints")
INSID3_MODEL_SIZE = "base"   # small / base / large
INSID3_CKPT = os.path.join(INSID3_CKPT_DIR, "dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth")
INSID3_IMAGE_SIZE = 768      # 输入分辨率，越小越快（768 兼顾速度与精度）
# 闭集置信度下限：patch 与最优类别的平均余弦相似度低于此值判为背景丢弃。
# 保守起见先设 0.0（余弦相似度 >=0 才认作命中），后续可按实测调。
INSID3_MIN_CONF = 0.0

# 参考图根目录：本文件同级 reference_views/ 下每个物体的 6 视角渲染图。
# （Gazebo 无背景渲染截图，比早期 UV 贴图更接近实拍视角。）
REF_MODELS_ROOT = os.path.join(os.path.dirname(os.path.realpath(__file__)), "reference_views")

# 每个物体 6 个视角的文件名（前/后/左/右/上/下），全部用作该类参考原型。
REF_VIEW_NAMES = ["front", "back", "left", "right", "top", "bottom"]

# 目标类（label 与 vision_pipeline.ITEM_NAMES 对齐，保证 classify_phrase 命中）。
TARGET_CLASSES = ["apple", "coke can", "bowl", "banana"]

# 闭集类别集合：label -> reference_views 下的子目录名（含 6 视角渲染图）。
# 目标类 + 干扰类（仿真里可能出现的其它 YCB 物体）全部纳入，用于剔除干扰。
INSID3_CLASSES = {
    # ── 目标类 ──
    "apple":    "apple",
    "coke can": "coke_can",
    "bowl":     "bowl",
    "banana":   "banana",
    # ── 干扰类 ──
    "cracker box":    "cracker_box",
    "sugar box":      "sugar_box",
    "chips can":      "chips_can",
    "beer":           "beer",
    "bleach cleanser": "bleach_cleanser",
    "mustard bottle": "mustard_bottle",
    "pudding box":    "pudding_box",
    "tomato soup can": "tomato_soup_can",
    "master chef can": "master_chef_can",
    "tuna fish can":  "tuna_fish_can",
    "windex bottle":  "windex_bottle",
    "gelatin box":    "gelatin_box",
    "potted meat can": "potted_meat_can",
    "pitcher base":   "pitcher_base",
}

_HUB_NAMES = {"small": "dinov3_vits16", "base": "dinov3_vitb16", "large": "dinov3_vitl16"}


class Insid3Reviewer:
    """用冻结 INSID3（DINOv3 骨干 + 去偏置）对候选框 patch 做闭集分类复核。"""

    def __init__(self, device=None):
        import torch

        # 惰性 import INSID3 模型类（真正的 visinf/INSID3，非 CLIP 替换）
        from models.insid3 import INSID3

        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")

        # 权重文件须存在且体积正常（官方门控下载失败时会留下 111 字节的
        # AccessDenied XML，体积阈值可把这种占位垃圾识别为“缺失”）。
        if not os.path.exists(INSID3_CKPT) or os.path.getsize(INSID3_CKPT) < 1024 * 1024:
            raise FileNotFoundError(
                "INSID3/DINOv3 骨干权重缺失或损坏: {}\n"
                "请先下载 dinov3_vitb16_pretrain_lvd1689m-73cec8be.pth 放入该目录"
                "（官方门控，见文件头部注释）。".format(INSID3_CKPT)
            )

        # 构建冻结的 DINOv3 骨干 + INSID3 包装（复用 INSID3 的位置偏置去相关）。
        # 直接 import dinov3_vitb16，避开 torch.hub.load——后者会执行整个
        # hubconf.py，连带 import dinov3.hub.dinotxt/depthers/... 引入 torchmetrics。
        from dinov3.hub import backbones as _dinov3_backbones
        encoder = getattr(_dinov3_backbones, _HUB_NAMES[INSID3_MODEL_SIZE])(weights=INSID3_CKPT)
        self.model = INSID3(
            encoder=encoder,
            image_size=INSID3_IMAGE_SIZE,
            device=self.device,
        ).to(self.device)
        self.model.eval()
        # 骨干全部冻结，只推理，禁止任何训练/微调。
        for param in self.model.parameters():
            param.requires_grad = False

        # 归一化（ToTensor + Normalize，不含 Resize 拉伸）。细长物体（香蕉/可乐罐）
        # 若被 build_transform 的 Resize((S,S)) 硬拉成正方形会毁掉形状，这里只保留
        # 归一化，几何变换改由 _pad_to_square 保长宽比 + 补灰边完成。
        from torchvision import transforms
        self._norm = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
        ])

        # 预计算各类参考原型（只算一次）。
        self.class_labels = list(INSID3_CLASSES.keys())
        self.prototypes = self._build_prototypes()

    # ── 参考原型 ──────────────────────────────────────────────

    @staticmethod
    def _pad_to_square(pil, size):
        """保长宽比缩到能放进 size×size，再四周补灰边（fill=246）。

        灰边与参考渲染图的浅灰背景一致（>=200），使其在 _align_fg_mask 的
        raw<200 前景判断里被判为背景。返回 (padded_pil, (new_w,new_h,pad_left,pad_top))。
        """
        from PIL import Image
        import torchvision.transforms.functional as TF

        W, H = pil.size
        scale = min(size / W, size / H)
        new_w = max(1, int(round(W * scale)))
        new_h = max(1, int(round(H * scale)))
        img = pil.resize((new_w, new_h), Image.BILINEAR)
        pad_left = (size - new_w) // 2
        pad_top = (size - new_h) // 2
        pad_right = size - new_w - pad_left
        pad_bottom = size - new_h - pad_top
        img = TF.pad(img, (pad_left, pad_top, pad_right, pad_bottom), fill=246)
        return img, (new_w, new_h, pad_left, pad_top)

    @staticmethod
    def _align_fg_mask(fg, new_w, new_h, pad_left, pad_top, h, w, size):
        """把原图前景掩码 fg(H,W) 按补边成方的几何对齐到特征图 (h,w) 布尔掩码。"""
        fg_resized = cv2.resize(fg, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((size, size), dtype=np.uint8)
        canvas[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = fg_resized
        return cv2.resize(canvas, (w, h), interpolation=cv2.INTER_NEAREST) > 0

    def _load_ref_tensor(self, path):
        """读取参考图并套用「补边成方」预处理，返回 (tensor, geometry)。

        tensor: (1, C, S, S) 归一化张量；geometry: (new_w, new_h, pad_left, pad_top)，
        供 _build_prototypes 把前景掩码对齐到同一几何。
        """
        import torch
        from PIL import Image

        img = Image.open(path).convert("RGB")
        padded, geom = self._pad_to_square(img, INSID3_IMAGE_SIZE)
        t = self._norm(padded).unsqueeze(0)
        return t.to(self.device), geom

    def _build_prototypes(self):
        """对每个类别：提取去偏置特征，在前景区域池化得到 L2 归一化原型 (C,)。

        每个类别有多张参考图（6 个视角），分别计算原型后取平均，
        得到单一类原型，比单视角更稳。
        """
        import torch
        import torch.nn.functional as F

        prototypes = []
        for label in self.class_labels:
            subdir = os.path.join(REF_MODELS_ROOT, INSID3_CLASSES[label])
            if not os.path.isdir(subdir):
                raise FileNotFoundError("INSID3 参考图目录缺失: {}".format(subdir))

            view_protos = []
            for view in REF_VIEW_NAMES:
                path = os.path.join(subdir, view + ".png")
                if not os.path.exists(path):
                    continue

                ref_tensor, geom = self._load_ref_tensor(path)   # (1, C, S, S) + geometry
                fmaps = self.model._extract_features(ref_tensor.unsqueeze(1))   # (1,1,C,h,w)
                fmaps_norm = F.normalize(fmaps, p=2, dim=2)
                feats = self.model._debias_features(fmaps_norm)[0, 0]           # (C,h,w)

                # 前景掩码：渲染图背景为浅灰（~246），物体更暗，按亮度剔除背景。
                raw = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY)
                h, w = feats.shape[-2:]
                fg = (raw < 200).astype(np.uint8)
                new_w, new_h, pad_left, pad_top = geom
                fg = self._align_fg_mask(fg, new_w, new_h, pad_left, pad_top,
                                         h, w, INSID3_IMAGE_SIZE)
                fg = torch.from_numpy(fg).to(feats.device)                     # (h,w) bool

                flat = feats.reshape(feats.shape[0], -1)                       # (C, h*w)
                fg_flat = fg.reshape(-1)
                if fg_flat.any():
                    proto = flat[:, fg_flat].mean(dim=1)
                else:
                    proto = flat.mean(dim=1)
                view_protos.append(F.normalize(proto, p=2, dim=0))

            if not view_protos:
                raise FileNotFoundError("INSID3 参考图缺失（{} 目录无任何视角）".format(subdir))
            # 多视角原型取平均后再归一化，作为该类单一原型。
            cls_proto = torch.stack(view_protos, dim=0).mean(dim=0)            # (C,)
            prototypes.append(F.normalize(cls_proto, p=2, dim=0))

        return torch.stack(prototypes, dim=0)   # (K, C)

    # ── 主接口 ────────────────────────────────────────────────

    def review(self, img_bgr, boxes_xyxy):
        """对候选框做闭集复核，返回 (保留下标, 保留标签)。

        Args:
            img_bgr     : 原始 BGR 图像 (H, W, 3)。
            boxes_xyxy  : torch.Tensor (N, 4) 或 np.ndarray (N, 4) 像素坐标 [x1,y1,x2,y2]。
        Returns:
            keep_idx    : list[int]，复核通过的框在输入数组中的下标。
            keep_labels : list[str]，对应每个框的最终类别（目标类之一）。
        """
        import torch

        if boxes_xyxy is None or len(boxes_xyxy) == 0:
            return [], []

        boxes_np = boxes_xyxy.detach().cpu().numpy() if torch.is_tensor(boxes_xyxy) \
            else np.asarray(boxes_xyxy)
        patches, valid_idx, geoms = self._crop_patches(img_bgr, boxes_np)
        if len(patches) == 0:
            return [], []

        # 批量提取去偏置特征（一次前向）。拼成 (1, M, C, H, W)：B=1、T=M，
        # 与 INSID3 内部 predict_mask 的用法一致。
        import torch.nn.functional as F
        imgs = torch.cat(patches, dim=0).unsqueeze(0)            # (1, M, C, H, W)
        fmaps = self.model._extract_features(imgs)               # (1, M, C, h, w)
        # 与 _build_prototypes 一致：先 L2 归一化再去偏置。
        fmaps_norm = F.normalize(fmaps, p=2, dim=2)
        feats = self.model._debias_features(fmaps_norm)[0]       # (M, C, h, w)
        M, C, h, w = feats.shape

        # 只在「物体区域」池化（剔除补边），避免细长物体被大片灰边稀释。
        # 与参考原型「只在前景池化」对齐；每框区域 = 补边前的内容矩形。
        obj_mask = torch.zeros((M, h, w), dtype=torch.bool, device=feats.device)
        sx = w / INSID3_IMAGE_SIZE
        sy = h / INSID3_IMAGE_SIZE
        for j, (new_w, new_h, pad_left, pad_top) in enumerate(geoms):
            x0 = int(round(pad_left * sx)); x1 = int(round((pad_left + new_w) * sx))
            y0 = int(round(pad_top * sy));  y1 = int(round((pad_top + new_h) * sy))
            obj_mask[j, y0:y1, x0:x1] = True

        pooled = (feats * obj_mask.unsqueeze(1)).sum(dim=(2, 3))          # (M, C)
        counts = obj_mask.sum(dim=(1, 2)).float().clamp(min=1.0)          # (M,)
        pooled = pooled / counts.unsqueeze(1)                             # (M, C)

        # 与各类原型做余弦相似度（二者已 L2 归一化），argmax。
        sims = torch.einsum("mc,kc->mk", pooled, self.prototypes)         # (M, K)
        best_sim, best_idx = sims.max(dim=1)                     # (M,)

        keep_idx, keep_labels = [], []
        for j, (s, k) in enumerate(zip(best_sim, best_idx)):
            label = self.class_labels[int(k)]
            if label not in TARGET_CLASSES:
                continue                       # 干扰类 → 丢弃
            if float(s) < INSID3_MIN_CONF:
                continue                       # 背景/未知 → 丢弃
            keep_idx.append(valid_idx[j])      # 回映射到原始框下标
            keep_labels.append(label)

        return keep_idx, keep_labels

    # ── 工具 ──────────────────────────────────────────────────

    def _crop_patches(self, img_bgr, boxes_np):
        """按框从原图 crop 出 patch，套用「补边成方」预处理。

        返回 (patches, valid_idx, geoms)：valid_idx 为成功 crop 的框在原数组中的下标，
        geoms 为每个 patch 的 (new_w, new_h, pad_left, pad_top)，供 review() 把
        特征池化限制在物体区域（剔除补边），保证与特征、原始框一一对应。
        """
        import torch
        from PIL import Image

        h, w = img_bgr.shape[:2]
        patches = []
        valid_idx = []
        geoms = []
        for i, box in enumerate(boxes_np):
            x1, y1, x2, y2 = [int(round(v)) for v in box]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            crop_bgr = img_bgr[y1:y2, x1:x2]
            crop_rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
            pil = Image.fromarray(crop_rgb)
            padded, geom = self._pad_to_square(pil, INSID3_IMAGE_SIZE)
            t = self._norm(padded).unsqueeze(0)                  # (1, C, S, S)
            patches.append(t.to(self.device))
            geoms.append(geom)
            valid_idx.append(i)
        return patches, valid_idx, geoms


# 供离线自测：python3 insid3_review.py <图片路径>
def main():
    import cv2
    import numpy as np

    if len(sys.argv) < 2:
        print("用法: python3 insid3_review.py <图片路径>")
        sys.exit(1)

    img_bgr = cv2.imread(sys.argv[1])
    if img_bgr is None:
        print("无法读取图片: {}".format(sys.argv[1]))
        sys.exit(1)

    reviewer = Insid3Reviewer()
    # 用整图作为单个候选框自测（实际由 vision_pipeline 传入检测框）。
    h, w = img_bgr.shape[:2]
    boxes = np.array([[0, 0, w, h]], dtype=np.float32)
    keep_idx, keep_labels = reviewer.review(img_bgr, boxes)
    print("复核结果: {} 个框通过".format(len(keep_idx)))
    for i, l in zip(keep_idx, keep_labels):
        print("  {} box={}".format(l, [int(v) for v in boxes[i]]))


if __name__ == "__main__":
    main()

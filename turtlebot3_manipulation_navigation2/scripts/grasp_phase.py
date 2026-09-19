#!/usr/bin/env python3
"""
抓取阶段（Phase 2）—— **唯一实现**，两个入口共用：

    patrol_task.py       比赛主流程：Phase 1（巡逻计数 + 写答案 JSON）跑完 → 调 GraspPhase
    dining_grasp_task.py 独立调试入口（薄壳）：只做参数解析 → 调同一个 GraspPhase

为什么不各写一份：旧工作区的 dining_grasp_task.py 是独立驱动，而规则书 3.1 要求
"基础题做完不回起点、直接从客厅去餐厅" → 抓取必须并进主流程。两份逻辑各自漂移是坑 ✗

流程（每个位置各司其职）：
    ① 观察位（距桌心 ~1.15 m，能看到整张桌子）：调 /detect_grasp_target → 选一个可夹的目标
       （规则书 3.2/3.4：桌上四个物品任选一个），其余物品留作 obstacles
       ★ 观察位与站位的【朝向】都由 支撑面（餐桌）的桌沿法线 定，不由"机器人当前位姿"定：
         这样车头垂直桌沿、物体在机械臂正前方（"对正桌子"），
         也免得 AMCL 的定位误差 + Nav2 停位偏差把停车角度带偏几度~二十几度 ✗
         （实测踩过：按"机器人当前方位"算站位 → 站位 yaw = −1.155 rad，
          比桌沿法线 −1.571 偏 24° → 机械臂斜着伸到桌上）
    ② 站位（物体正前方 0.33~0.39 m，yaw = 法线反方向）→ 导航过去
    ③ 站位上【重新测一次】（相机与桌面等高，站位上整只物体可见 ✓），
       用相对量做 creep 微调，把物体开到 base_footprint(0.33, 0)
    ④ 调 /grasp_fixed_object（target + obstacles）；BAD_TARGET/NO_SOLUTION 就换下一个目标

★ 只用视觉（相机 + GroundingDINO/SAM2/INSID3 复核），**不读 gz 真值**：
  规则书禁止读仿真真值；本文件里也没有任何 `ign/gz model` 调用、没有"读不到就退真值"的退路。
  唯一的外部配置来源是 objects.yaml / grasp_params.yaml（碰撞箱尺寸、支撑面几何）。

★ 本模块**绝不调用 rclpy.spin\***：它在 patrol_task 的动作回调链里运行，
  再 spin 会死锁。所有等待都是"轮询 + time.sleep"，
  依赖调用方使用 MultiThreadedExecutor（订阅/服务回调在别的线程继续跑）。
"""

import math
import os
import time

import rclpy
import tf2_ros
from rclpy.action import ActionClient
from rclpy.time import Time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav2_msgs.action import NavigateToPose

from turtlebot3_manipulation_grasp.msg import GraspTargetStamped
from turtlebot3_manipulation_grasp.srv import DetectGraspTarget, GraspFixedObject

# ═══════════════════════════════════════════════════════════════
# 固定场景常量（map 帧 = Gazebo world 帧）
# ═══════════════════════════════════════════════════════════════

# 初始位姿（与 turtlebot3_franka.launch.py spawn 参数、patrol_task 一致）
INITIAL_X, INITIAL_Y, INITIAL_YAW = -5.30, -0.50, 0.0


# ── 观察位 / 站位的朝向：一律用【支撑面桌沿的法线】 ────────────────
# 桌沿法线（map 帧）：桌面在本场景里是轴对齐矩形（TABLE_YAW = ±π/2），
# 所以 4 条边对应 map 的 ±x / ±y 四个方向。选哪一条 = "机器人现在这一侧"，
# 判据只用【物体 → 机器人的方向】（不要求准，AMCL 差十几度也不影响选边）。
TABLE_FACE_NORMALS = ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0))
# 相邻两张餐桌（世界文件 example.world 的静态位姿，yaw=π/2 → 世界系半尺寸 x±0.6 / y±0.25）：
# 相机水平视野 62°，站在本桌前会把它们一起看进来 → 它们桌上的物体必须被判"不在本桌"
NEIGHBOR_TABLES = ((1.5, 2.0), (2.7, 1.5))     # dinning_table_1 / dinning_table_2
NEIGHBOR_HALF = (0.6, 0.25)
OBSERVATION_DIST = 1.15     # m，观察位到桌心的距离（Nav2 停偏 0.4 m 也还在画面内）
# ★ "近看"位：观察位太远时开集检测的置信度会掉到阈值以下 ✗
#   实测（2026-09-14）：站在观察位（离桌心 1.15 m、物体 0.9~1.5 m）时 7 个目标的
#   置信度只有 0.30~0.47 < 0.50，一个都不够格；而贴到站位（离物体 0.4 m）时同样是
#   mustard_bottle/sugar_box 能到 0.84 ✓。物体在 640x480 画面里只有几十像素 →
#   置信度是【距离】的函数。所以观察位一个都不够格时，再靠近到近看位重看一次。
CLOSE_LOOK_DIST = 0.68      # m，近看位到桌心的距离（≈ 桌沿外 0.43 m、离物体 0.7~0.9 m）
CLOSE_MIN_CONFIDENCE = 0.25 # 近看这一趟放宽到这里（配合适配层的尺寸核对防误检）

# 臂基座（fr3_link0）在 base_footprint 下的 x 偏移（实测）。只用来看够不够得着。
ARM_BASE_X = -0.092

# 抓取服务返回的 stage → 人类可读（与 srv/GraspFixedObject.srv 的常量一一对应）
STAGE_TEXT = {
    0: "OK 完成",
    1: "NO_TARGET 没有可用目标",
    2: "BAD_TARGET 目标不合法（查表/桌面/可达性校验没过）",
    3: "SCENE_FAILED 场景没建起来（move_group / TF）",
    4: "INIT_FAILED MTC 任务初始化失败（配置问题）",
    5: "NO_SOLUTION 规划无解",
    6: "EXEC_FAILED 执行失败（控制器 / 碰撞）",
    7: "BUSY 已有抓取在执行",
}

# ── 目标优先级（见 docs/handoff/HANDOFF_grasp_orchestration_design.md §2）─────────
# 越靠前越稳：圆柱类位置换算误差≈0；方盒有 8~20 mm 的【径向】偏差 + 掀翻风险；
# 香蕉长边 0.198 必须跨窄边、薄件指尖容易骑到顶面 → 放最后。
TARGET_TIERS = [
    ["coke can", "tomato_soup_can", "chips_can"],                  # 圆柱
    ["apple", "potted_meat_can"],                                  # 球 / 矮罐
    ["cracker_box", "mustard_bottle", "bleach_cleanser", "sugar_box"],  # 高瘦方盒
    ["banana", "gelatin_box"],                                     # 长条 / 薄件
]
# 夹爪开口 0.08 m，这些类别的窄边 ≥ 0.08 → 物理上夹不住（见 objects.yaml）
NON_GRASPABLE = {"bowl", "pudding_box", "tuna_fish_can", "master_chef_can",
                 "pitcher_base", "beer", "windex_bottle"}

# ── 站位与 creep ─────────────────────────────────────────────────
STANDOFF_DIST = 0.33        # m，车心到物体（实测可稳定顶抓的几何）
# 相机在 base_footprint 前方多远（URDF: camera_joint x=0.073 + rgb 0.003 ≈ 0.076）
CAMERA_FWD = 0.08
# 相机碰撞盒在 0.7865~0.8135 m，桌面顶 0.795 m → 相机若伸到桌面上方就会撞桌板 ✗
# 所以站位要保证"相机仍在桌沿外"：g ≥ 物体纵深 + CAMERA_FWD + 余量
STANDOFF_EDGE_MARGIN = 0.05
EDGE_CLEARANCE = 0.32       # m，车心到桌沿的最小距离（车半径 0.28 + 余量；涂黑桌块后东/北站位余量从 15→48）
CREEP_GOAL_XY = (0.33, 0.0)  # 物体应落到的 base_footprint 位置
# ★ 容差按"指尖夹持窗口 ±6.5 mm"定（合爪前开口 0.08 − 罐宽 0.067）：
#   原来 bearing tol = 0.05 rad → 在 0.33 m 处等于 ±16.5 mm，比窗口大一倍多 ✗
#   方位容差 0.013 rad ≈ ±4.3 mm ✓；距离（径向）宽容，因为物体在该方向自身就有几 cm
CREEP_TOL = 0.008
CREEP_BEARING_TOL = 0.013
CREEP_OK_ROUNDS = 3         # 连续 N 轮都在容差内才收工（防被单帧噪声骗停）
CREEP_V_MAX, CREEP_W_MAX = 0.12, 0.60
CREEP_KV, CREEP_KW = 1.2, 1.5
CREEP_SPIN_BEARING = 0.35
CREEP_TIMEOUT = 45.0        # s（视觉每次 1~5 s，收紧容差后轮数变多，预算放宽）
CREEP_DT = 0.10

# ── ★ 抓取点【自检】阈值（2026-09-18 实跑两次对比定出来的）──────────────
# 底盘在"微调收敛"和"抓取点重拍"之间**没动过**，所以这两次对同一个静止物体的
# 测量本该一致；不一致就说明至少有一次是错的 → 不能照着它下爪。
# ★ 必须【分方向】判，看总差异大小是判不出来的 —— 两次实跑的实测：
#     成功那次: 微调(0.377,+0.000) vs 重拍(0.402,+0.000) → 差 **25 mm**，但全在【前向】
#     失败那次: 微调(0.407,-0.000) vs 重拍(0.412,-0.019) → 差 20 mm，其中【横向 19 mm】
#   前向差只意味着"多伸 2 cm"，罐子仍在两指之间 ✓ 能夹住；
#   横向差直接决定两指能不能合到物体 —— 失败那次两指合到 57.0 mm **空合**（TF 间隙
#   76→57 全程无阻挡，而罐子窄边 66 mm），就是横向偏了 19 mm 造成的 ✗
#   ⇒ 横向容差必须严（按夹持窗口 ±6.5 mm 的量级给），前向可以松。
CONSIST_TOL_Y = 0.012       # 横向（两指闭合方向）容差 —— 严
# ★ 距离容差必须**松于** `_nudge` 的触发线（0.030），否则自检会抢在 nudge 之前
#   把"其实能补回来"的距离差直接拒掉 ✗ —— 距离差有 `_nudge` 专门负责（它本来就是
#   干这个的：驱动底盘把物体重新推到 standoff）。自检只在距离差**大到 nudge 也救不回**
#   时才拦。
CONSIST_TOL_R = 0.060       # 前向/距离容差 —— 松（> nudge 的 0.030）
CONSIST_TRIES = 2           # 自检不过就重拍，最多再拍 2 次
# ★ 重拍**救不回来**（2026-09-18 实跑证明）：三次重拍返回的是逐位相同的
#   base(+0.432,-0.025) conf=0.52 —— 这个视觉是**确定性的**，不是逐帧随机的。
#   重拍只值 15 秒的确认（"它确实稳定地这么认为"），不能指望它换个答案。
#   ⇒ 自检不过的实际含义是【这一轮的观测/定位本身有问题】，应当重来一轮，
#     而不是指望同一位置重拍。保留 CONSIST_TRIES>0 只为区分"偶发"和"稳定"。
MIN_GRASP_CONF = 0.70       # 置信度【仅提示】阈值：低于它只在日志里提一句，**不拒抓**
# ★ 别把它当闸门用 —— 四次实跑：夹住 0.89；空合 0.90 和 0.53（还有一次没记 conf）。
#   置信度和成败没有对应关系，而本布置正常检测就落在 0.52~0.62。

CMD_VEL_TOPIC = "/cmd_vel"  # navigation.launch.py 的 cmd_vel_relay 转到 /diff_controller

# ── 服务与超时 ───────────────────────────────────────────────────
VISION_SERVICE = "/detect_grasp_target"
GRASP_SERVICE = "/grasp_fixed_object"
NAV_ACTION = "/navigate_to_pose"
SERVICE_WAIT = 15.0
# ★ 视觉服务单次调用预算（2026-09-14 现场踩过）：
#   抓取模式 = 3 帧（18 类开集 + INSID3 闭集复核，每帧 ~15.5 s）+ 窄词表复核(11 s)
#   ≈ 60~75 s ⇒ 原来 60 s 的预算**经常超时** ✗
#   超时的后果很隐蔽：驱动退回"观察位（1 m 外）那次的估计"⇒ 抓取点偏十几厘米 ✗
#   ⇒ 机械臂看着"停在物体旁"却夹不到（现场就是这个现象）
VISION_CALL_TIMEOUT = 150.0     # 视觉单次调用预算（含 3 帧投票 + 窄词表复核）
GRASP_SERVICE_TIMEOUT = 900.0
MAX_NAV_RETRIES = 6
GOAL_ACCEPT_TIMEOUT = 15.0
TF_RETRY = 30
MIN_CONFIDENCE = 0.35           # 低于它不当作候选（开集检测在 1 m 外只有 0.35~0.46，
                                # 定 0.50 会把真目标全部拒掉 ✗；误检靠尺寸核对 +
                                # 支撑面校验 + 多帧投票压住，见 docs/handoff/HANDOFF_vision_grasp_interface.md）
REACH_MIN, REACH_MAX = 0.10, 0.75
MAX_TARGET_TRIES = 4            # 抓失败换目标的次数（规则书：四个里任选一个）。
                                # ★ 2026-09-18：按【抓取清单】逐个试、四个都试完。
                                #   原来是 2 —— 等于第一好抓的物体没成就只剩一次机会就收场 ✗
SAME_TARGET_RETRY = 2           # 同一目标"两指没夹到(空合/没合到)"后重新拍照识别重新抓的上限


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_rotate(q, v):
    qx, qy, qz, qw = q
    vx, vy, vz = v
    c = (qy * vz - qz * vy, qz * vx - qx * vz, qx * vy - qy * vx)
    c2 = (qy * c[2] - qz * c[1], qz * c[0] - qx * c[2], qx * c[1] - qy * c[0])
    return (vx + 2 * qw * c[0] + 2 * c2[0],
            vy + 2 * qw * c[1] + 2 * c2[1],
            vz + 2 * qw * c[2] + 2 * c2[2])


def find_grasp_config(name):
    """找抓取包的 config/<name>（objects.yaml / grasp_params.yaml）。

    本脚本在 nav2 包里，配置在 grasp 包里 → 先看源码树，再按 AMENT_PREFIX_PATH 找 share。
    """
    here = os.path.dirname(os.path.realpath(__file__))
    cands = [os.path.join(os.path.dirname(os.path.dirname(here)),
                          "turtlebot3_manipulation_grasp", "config", name)]
    for prefix in (os.environ.get("AMENT_PREFIX_PATH") or "").split(os.pathsep):
        if prefix:
            cands.append(os.path.join(prefix, "share", "turtlebot3_manipulation_grasp",
                                      "config", name))
    for c in cands:
        if os.path.isfile(c):
            return c
    return None


def load_close_squeeze(default=0.0045):
    """读 grasp_params.yaml 的 close_hand_squeeze（驱动侧只用它算"预期合爪间隙"）。

    ★ 必须读文件而不是在驱动里再写一套公式：C++ 与驱动各写一套就会漂移 ✗
    """
    path = find_grasp_config("grasp_params.yaml")
    if not path:
        return default
    try:
        import yaml
        doc = yaml.safe_load(open(path, encoding="utf-8")) or {}
        node = (doc.get("/**") or {}).get("ros__parameters") or {}
        v = node.get("close_hand_squeeze")
        return float(v) if v is not None else default
    except Exception:
        return default


def load_object_sizes():
    """读 objects.yaml 的三维尺寸 → {class_id: (depth, width, height)}（读不到返回 {}）。

    用途：驱动侧要知道"这次合爪应该合到多少"，才能判断实测行程是不是夹到了物体 ✓
    """
    path = find_grasp_config("objects.yaml")
    if not path:
        return {}
    try:
        import yaml
        doc = yaml.safe_load(open(path, encoding="utf-8")) or {}
        out = {}
        for k, v in (doc.get("objects") or {}).items():
            if isinstance(v, dict) and all(x in v for x in ("depth", "width", "height")):
                out[k] = (float(v["depth"]), float(v["width"]), float(v["height"]))
        return out
    except Exception:
        return {}


# 支撑面（要抓的物体所在的那张桌子）：**从 grasp_params.yaml 读**，不再写死 ✓
#   为什么必须同源：抓取侧（grasp_node）也是读这个文件 ✓，两处各写一份就会漂移 ✗
#   （原来这里硬编码 dinning_table_3，换张桌子就得改两处代码 ✗）
def load_support_surface():
    """→ (center_xyz, yaw, dims, half_x, half_y, top_z)；读不到就用餐厅那张桌子。"""
    cx, cy, cz, yaw, dims = 2.7, 2.0, 0.765, math.pi / 2, (0.5, 1.2, 0.03)
    path = find_grasp_config("grasp_params.yaml")
    if path:
        try:
            import yaml
            doc = yaml.safe_load(open(path, encoding="utf-8")) or {}
            n = (doc.get("/**") or {}).get("ros__parameters") or {}
            pose = n.get("support_surface.pose")
            if pose and len(pose) >= 3:
                cx, cy, cz = float(pose[0]), float(pose[1]), float(pose[2])
                yaw = float(pose[5]) if len(pose) > 5 else 0.0
            if n.get("support_surface.length") and n.get("support_surface.width"):
                dims = (float(n["support_surface.length"]),
                        float(n["support_surface.width"]),
                        float(n.get("support_surface.thickness", 0.03)))
        except Exception:
            pass
    half_x, half_y = (dims[1] / 2.0, dims[0] / 2.0) if abs(yaw) > 1.0 else (dims[0] / 2.0, dims[1] / 2.0)
    return (cx, cy, cz), yaw, dims, half_x, half_y


TABLE_CENTER_XYZ, TABLE_YAW, TABLE_DIMS, TABLE_HALF_X, TABLE_HALF_Y = load_support_surface()
TABLE_TOP_Z = TABLE_CENTER_XYZ[2] + TABLE_DIMS[2] / 2.0     # 桌面顶
TABLE_IS_DINING = (abs(TABLE_CENTER_XYZ[0] - 2.7) < 0.2 and abs(TABLE_CENTER_XYZ[1] - 2.0) < 0.2)


def load_graspable():
    """读 objects.yaml 的 graspable 标志 → {class_id: bool}（读不到返回 {}）。

    为什么要读它而不是在代码里写死名单：能夹什么由**碰撞箱尺寸 + 夹爪开口**决定，
    那是 objects.yaml 的职责（抓取侧查表用的也是它）→ 一处维护，别两处 ✗
    """
    path = find_grasp_config("objects.yaml")
    if not path:
        return {}
    try:
        import yaml
        doc = yaml.safe_load(open(path, encoding="utf-8")) or {}
        return {k: bool(v.get("graspable", False))
                for k, v in (doc.get("objects") or {}).items() if isinstance(v, dict)}
    except Exception:
        return {}


# ═══════════════════════════════════════════════════════════════
# 导航地图（判"这个点是不是空闲空间"）
# ═══════════════════════════════════════════════════════════════
# ★ 为什么要读地图：选"从哪条桌沿接近"时，光看"机器人现在在物体哪一侧"是不够的 ✗
#   实测踩过：机器人从起点 (-5.30,-0.50) 出发时，物体方向是"西边" → 选西侧桌沿
#   → 观察位算成 (1.55, 2.0)，而那里是 dining_table_1 的桌面里 ✗✗
#   （餐桌是并排的：table_1 x∈[0.9,2.1]、table_2 y∈[1.25,1.75]、table_3 在中间）
#   地图里这些桌子都是障碍 → 直接查地图就知道哪一面站得下人 ✓

ROBOT_CLEAR_OBS = 0.32      # m，观察位要求周围这么空（车半径 0.28 + 余量）
ROBOT_CLEAR_PARK = 0.10     # m，站位只要求"别压在桌子上/墙里"（贴桌沿停车是常态）


def find_nav_map():
    """找导航地图 map.yaml（本包 map/ 下；装好的 share 里也找一遍）。"""
    here = os.path.dirname(os.path.realpath(__file__))
    cands = [os.path.join(os.path.dirname(here), "map", "map.yaml")]
    for prefix in (os.environ.get("AMENT_PREFIX_PATH") or "").split(os.pathsep):
        if prefix:
            cands.append(os.path.join(prefix, "share", "turtlebot3_manipulation_navigation2",
                                      "map", "map.yaml"))
    for c in cands:
        if os.path.isfile(c):
            return c
    return None


class NavMap:
    """占用栅格（map.yaml + map.pgm），只用来回答"点 (x,y) 周围空不空"。"""

    def __init__(self, yaml_path):
        import yaml
        doc = yaml.safe_load(open(yaml_path, encoding="utf-8")) or {}
        self.res = float(doc["resolution"])
        self.ox, self.oy = float(doc["origin"][0]), float(doc["origin"][1])
        self.negate = bool(doc.get("negate", 0))
        self.occupied_thresh = float(doc.get("occupied_thresh", 0.65))
        path = os.path.join(os.path.dirname(yaml_path), doc["image"])
        with open(path, "rb") as fh:
            blob = fh.read()
        # 手写 P5 解析（不依赖 PIL）：magic / 宽 高 / maxval，其间可能有 '#' 注释
        fields, pos = [], 0
        while len(fields) < 4:
            while blob[pos:pos + 1].isspace():
                pos += 1
            if blob[pos:pos + 1] == b"#":
                while blob[pos:pos + 1] not in (b"\n", b""):
                    pos += 1
                continue
            start = pos
            while not blob[pos:pos + 1].isspace():
                pos += 1
            fields.append(blob[start:pos])
        if fields[0] not in (b"P5", b"P2"):
            raise ValueError("只支持 P5/P2 PGM: " + repr(fields[0]))
        self.w, self.h, maxval = int(fields[1]), int(fields[2]), int(fields[3])
        pos += 1
        if fields[0] == b"P5":
            import numpy as np
            self.pix = np.frombuffer(blob[pos:pos + self.w * self.h], dtype="u1")
        else:
            import numpy as np
            self.pix = np.array(blob[pos:].split()[:self.w * self.h], dtype="u1")
        self.pix = self.pix.reshape(self.h, self.w).astype("float32") / float(maxval)

    def occupied(self, x, y):
        col = int((x - self.ox) / self.res)
        row = self.h - 1 - int((y - self.oy) / self.res)     # 图像第 0 行是地图最上方
        if col < 0 or row < 0 or col >= self.w or row >= self.h:
            return True                                     # 界外当障碍
        p = self.pix[row, col]
        occ = p if self.negate else 1.0 - p
        return occ > self.occupied_thresh

    def free(self, x, y, clearance=0.0):
        """(x,y) 周围 clearance 半径内没有一个占用栅格 → 空闲。"""
        if clearance <= 0.0:
            return not self.occupied(x, y)
        step = max(self.res, 0.05)
        n = int(math.ceil(clearance / step))
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                if math.hypot(i * step, j * step) > clearance + 1e-9:
                    continue
                if self.occupied(x + i * step, y + j * step):
                    return False
        return True


class GraspPhase:
    """抓取阶段：选目标 → 站位 → creep → 抓取。可被任何 Node 复用。"""

    def __init__(self, node, *, target_classes=None, min_confidence=MIN_CONFIDENCE,
                 nav_client=None, cmd_vel_topic=CMD_VEL_TOPIC):
        self.node = node
        self.log = node.get_logger()
        self.min_confidence = min_confidence
        # 想找哪些类别（空 = 不限，按 TARGET_TIERS 自己挑）
        self.target_classes = list(target_classes or [])
        self.tf_buffer = getattr(node, "_tf_buffer", None)
        if self.tf_buffer is None:
            # ★ 调用方可能没有 TF 缓冲（dining_grasp_task.py 就没有）→ 自己建一个。
            #   没有它就拿不到 map←base_footprint：观察位、支撑面校验、站位全部算不出来 ✗
            #   （实测：dining_grasp_task 直接报"拿不到 map←base_footprint，算不出观察位"）
            self.tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self.vision_client = node.create_client(DetectGraspTarget, VISION_SERVICE)
        self.grasp_client = node.create_client(GraspFixedObject, GRASP_SERVICE)
        self.cmd_pub = node.create_publisher(Twist, cmd_vel_topic, 10)
        # 复用调用方的 Nav2 客户端（patrol_task 里已经有），没有就自己建
        self.nav_client = nav_client or ActionClient(node, NavigateToPose, NAV_ACTION)
        self.last_targets = []       # 最近一次检测结果（给上层/证据用）
        # ★ 手指关节实测位置：判断"到底有没有夹到物体"的唯一直接证据
        #   （仿真里合爪是位置控制：夹到物体就会停在物体宽度处、到不了指令值 ✓）
        self._finger_q = None        # joint1（历史用法，= 2×它的"两指间距"只在对称时成立）
        self._finger_q2 = None       # joint2（2026-09-18 起两指独立驱动，必须分开看）
        self.object_sizes = load_object_sizes()
        self.close_squeeze = load_close_squeeze()
        # ★ 注意：GraspPhase 不是 Node（只是拿着调用方的 node）→ 必须用 self.node.* ✗
        self.node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._track = {}             # 类别 → (时间, odom 坐标)：creep 的里程计跟踪基准
        # ★ 抓取清单（2026-09-18）：开局把桌上可夹物体的【顺序 + map 初始位置】冻结一次，
        #   之后按序消耗；"已放弃"按【实例号】记（不再按 class_id，同类物体不连坐 ✓）
        self._plan = None
        self._plan_rebuilt = False   # 近看漏检后是否已重建过清单（只重建一次）
        # 能夹什么以 objects.yaml 为准（读不到才退回代码里的兜底名单）
        self.graspable = load_graspable()
        if self.graspable:
            ok = [k for k, v in self.graspable.items() if v]
            self.log.info("objects.yaml 里可夹类别 {} 个：{}".format(len(ok), ", ".join(ok)))
        else:
            self.log.warn("读不到 objects.yaml 的 graspable 标志，退回内置名单")
        # 导航地图：用来判"观察位/站位是不是空闲空间"（见 NavMap 的说明）
        self.nav_map = None
        mp = find_nav_map()
        if mp:
            try:
                self.nav_map = NavMap(mp)
                self.log.info("已载入导航地图 {}（{}x{} @ {:.2f} m/格）→ 站位/观察位会查空闲空间"
                              .format(mp, self.nav_map.w, self.nav_map.h, self.nav_map.res))
            except Exception as e:                       # noqa: BLE001
                self.log.warn("解析地图失败（{}: {}）→ 站位只按餐桌脚印判".format(
                    type(e).__name__, e))
        else:
            self.log.warn("找不到导航地图 → 站位只按餐桌脚印判（不查全局空闲空间）")

    def _free(self, x, y, clearance):
        return True if self.nav_map is None else self.nav_map.free(x, y, clearance)

    def _on_joint_state(self, msg):
        # ★ 2026-09-18：两指自独立驱动（mimic 已弃用）→ 两个都要采，不能 break ✗
        #   成功抓取实测：joint1=0.0285、joint2=0.0372（和 = 0.0657 ≈ 罐头窄边 0.066 ✓）
        #   —— 两指可以不对称，所以"2×joint1"不再等于真实间隙。
        for name, pos in zip(msg.name, msg.position):
            if name == "fr3_finger_joint1":
                self._finger_q = float(pos)
            elif name == "fr3_finger_joint2":
                self._finger_q2 = float(pos)

    def tcp_pose_base(self):
        """指尖平面 fr3_hand_tcp 在当前 base_footprint 系里的 (x, y, z)；取不到返回 None。

        ★ 这是"机械臂到底把指尖送到了哪"的唯一直接证据（TF = 机器人自身运动学，不是真值）：
          把它和视觉量到的物体顶面高度一比，就能立刻分辨
            · 指尖确实落在物体高度带里 → 问题在物体/模型尺寸
            · 指尖停在物体顶面之上   → 问题在机械臂/规划（下去不够深）✗
        """
        if self.tf_buffer is None:
            return None
        try:
            tr = self.tf_buffer.lookup_transform("base_footprint", "fr3_hand_tcp", Time())
            t = tr.transform.translation
            return (t.x, t.y, t.z)
        except Exception:
            return None

    def finger_gap_mm(self):
        """两指【真实间距】(mm)：直接量 fr3_leftfinger 与 fr3_rightfinger 两个帧的距离。

        ★ 这是合爪判据的**主证据**（2026-09-18 起）：
          mimic 已弃用、joint2 独立驱动 → 两指**可以不对称**（实测成功抓取
          joint1=0.0285 / joint2=0.0372），所以 `2×joint1` 不再等于真实间隙 ✗
          只有量两个手指帧的距离才是真正"两指之间有多宽" ✓
          （TF 是机器人自己的运动学，不是 gz 真值）
        """
        if self.tf_buffer is None:
            return None
        try:
            tr = self.tf_buffer.lookup_transform("fr3_leftfinger", "fr3_rightfinger", Time())
            t = tr.transform.translation
            return math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z) * 1000.0
        except Exception:
            return None

    def fingers_mm(self):
        """两指间距（mm）：转调 finger_gap_mm（TF 实测），读不到返回 None。

        ⚠️ 以前是 `2 × joint1` —— mimic 弃用后两指可不对称，那个式子**不再成立** ✗
           任何人想用"两指间距"都请走这条（= 真实测量），不要自己算 2×关节值。
        """
        return self.finger_gap_mm()

    # ══════════════ 等待原语（绝不 spin ✗）══════════════
    def sleep(self, seconds):
        """等一会儿。调用方必须是多线程 executor，否则时钟/回调不会前进。"""
        time.sleep(max(0.0, seconds))

    def wait_future(self, fut, timeout):
        """轮询 future（不 spin）。"""
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            if fut.done():
                return True
            time.sleep(0.02)
        return fut.done()

    def wait_client(self, client, timeout=SERVICE_WAIT):
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            if client.service_is_ready():
                return True
            time.sleep(0.05)
        return False

    # ══════════════ ① 目标获取（只有视觉；真值一律不读 ✗）══════════════
    def fetch_targets(self):
        """返回 GraspTargetStamped 列表（按置信度降序）。拿不到就返回空列表。

        ★ 只调视觉服务 /detect_grasp_target。以前这里有一条"读不到就退回 gz 真值"的
          退路 —— 已按用户要求**整条删除**：真值 = 直接读仿真答案，裁判禁止（即使是
          调试也不许用，因为它会让"视觉到底行不行"这个结论失效 ✗）。
          视觉拿不到目标时，本阶段就老老实实失败并打日志。
        """
        if not self.wait_client(self.vision_client):
            self.log.error("视觉服务 {} 不可用（适配层没起来？）".format(VISION_SERVICE))
            return []
        req = DetectGraspTarget.Request()
        req.class_ids = self.target_classes or []
        t_v0 = time.time()
        fut = self.vision_client.call_async(req)
        if not self.wait_future(fut, VISION_CALL_TIMEOUT):
            self.log.error("视觉服务调用超时（>{:.0f}s）".format(VISION_CALL_TIMEOUT))
            return []
        res = fut.result()
        if res is None:
            self.log.error("视觉服务无响应")
            return []
        if not res.success or not res.targets:
            self.log.warn("视觉没有可用目标：{}".format(res.message))
            return []
        tg = sorted(res.targets, key=lambda t: -t.confidence)
        self.log.info("视觉返回 {} 个目标（{}；耗时 {:.1f}s）".format(
            len(tg), res.message, time.time() - t_v0))
        return tg

    # ══════════════ ①b 支撑面桌沿法线（观察位与站位共用同一个朝向）══════════════
    def _face_ok(self, n, obj_map, check_reach=False):
        """从桌沿法线 n 那一侧接近，观察位与站位是否都站得下人。

        观察位要求 clearance 0.32 m（开阔地）；站位只要求 0.16 m
        （贴桌沿停车是常态，EDGE_CLEARANCE 另外由 standoff_pose 保证）。
        check_reach：**观察位必须传 False** ✗ —— 观察位用【桌心】当 obj_map，那只是
        "机器人从哪一侧来"的代理，跟"够不够得着"无关；只有抓取选面（obj_map 是真物体）
        才要验"这一面抓得到"。
        """
        ox = TABLE_CENTER_XYZ[0] + n[0] * OBSERVATION_DIST
        oy = TABLE_CENTER_XYZ[1] + n[1] * OBSERVATION_DIST
        if not self._free(ox, oy, ROBOT_CLEAR_OBS):
            return False
        for g in (STANDOFF_DIST, STANDOFF_DIST + 0.1, STANDOFF_DIST + 0.2, STANDOFF_DIST + 0.3):
            if self._free(obj_map[0] + n[0] * g, obj_map[1] + n[1] * g, ROBOT_CLEAR_PARK):
                break
        else:
            return False
        # ★ 够得着判据（2026-09-16）：creep 会把物体停到 base_footprint(goal_d, 0)，
        #   goal_d = max(0.33, 沿法线走到桌沿的距离 + 相机余量)。臂基座在 base 后 0.092 m，
        #   若 goal_d 太远则站位上 check_reach 会把这面的候选一轮轮拒光（东面对桌块深处的
        #   bowl 就够不着 ✗）→ 这里提前否掉这一面，让 approach_normal 改选别面。
        if check_reach:
            t_exit = self._table_exit_distance(obj_map, n[0], n[1])
            goal_d = max(STANDOFF_DIST, t_exit + CAMERA_FWD + STANDOFF_EDGE_MARGIN)
            if math.hypot(goal_d - ARM_BASE_X, 0.0) > REACH_MAX:
                return False
        return True

    def approach_normal(self, obj_map, robot_map, check_reach=False):
        """选一条"[物体] 朝 [机器人] 那一侧、而且真的站得下人"的桌沿外法线。

        ★ 为什么不能再用"机器人当前方位"当接近方向（原实现就是那么干的）：
          站位 yaw = 从站位指向物体的方位，而站位 = 物体 + g × (物体→机器人方向)。
          "物体→机器人"这个方向里含【AMCL 定位误差】和【观察位停位偏差】：
          实测机器人停在 (2.6, 2.93)（观察位是 (2.7, 3.2)），算出来
          yaw = −1.155 rad，而桌沿法线是 −1.571 rad → **机械臂斜 24° 伸到桌上** ✗
        ★ 也不能只看方位：机器人从起点 (-5.30,-0.50) 出发时"物体在西边"→ 选西侧桌沿，
          而西侧桌沿外是 dining_table_1 的桌面 ✗（实测观察位被算到 (1.55, 2.0) 桌子里）。
          所以顺序是：先按方位排序（少绕路），再拿地图逐条筛"站得下人" ✓
        """
        dx = robot_map[0] - obj_map[0]
        dy = robot_map[1] - obj_map[1]
        cands = sorted(TABLE_FACE_NORMALS, key=lambda n: -(n[0] * dx + n[1] * dy))
        for n in cands:
            if self._face_ok(n, obj_map, check_reach=check_reach):
                if n != cands[0]:
                    self.log.info("  桌沿法线：按方位首选 {} 站不下人 → 改用 {}".format(
                        tuple(cands[0]), tuple(n)))
                return n
        self.log.warn("四条桌沿都站不下人（地图没载入？）→ 退回按方位选 {}".format(tuple(cands[0])))
        return cands[0]

    def observation_pose_for(self, robot_map, dist=OBSERVATION_DIST):
        """观察位 = 桌心沿"机器人那一侧的法线"外推 dist，朝向 = 面对桌心。

        它与站位用的是**同一条法线** → 站在观察位上机械臂就已经对正桌子，
        到站位只是沿同一条直线靠近，不会中途"拧"过去。
        """
        n = self.approach_normal(TABLE_CENTER_XYZ, robot_map)
        px = TABLE_CENTER_XYZ[0] + n[0] * dist
        py = TABLE_CENTER_XYZ[1] + n[1] * dist
        yaw = math.atan2(TABLE_CENTER_XYZ[1] - py, TABLE_CENTER_XYZ[0] - px)
        return (px, py, yaw)

    # ══════════════ ② 选目标（硬过滤 + 档位 + 并列时排序）══════════════
    @staticmethod
    def _on_support(p_map, margin=0.15):
        """目标是否落在支撑面（餐桌 dinning_table_3）足迹内（带容差，吸收 AMCL 误差）。

        ★ 必须有这道校验：站在观察位面朝餐桌时，相机水平视野 62°，会**同时看到左右
          两张桌子** ✗（实测：table_1 x0.9~2.1、table_2 y1.25~1.75，都在画面里，
          它们上面的物体比本桌的多）。曾经因此选中 table_2 上的东西 →
          站位被算到 table_2 脚印【里面】(2.849,1.460) → **车直接开上桌子** ✗
        ★ 光靠"足迹 + 容差"不够（容差要放大到 0.20 m 才吃得下 AMCL 误差，而 table_1
          东端 chips_can 在 (1.9, 2.0)，|dx| = 0.8 正好落进 0.6+0.20 的窗里 ✗）
          → 再显式排掉【邻居餐桌自己的脚印】里的点 ✓
        """
        if TABLE_IS_DINING:                       # 只在餐厅那三张并排桌子时启用 ✓
            for cx, cy in NEIGHBOR_TABLES:        # 邻居桌子脚印内 → 不是本桌的目标
                if (abs(p_map[0] - cx) <= NEIGHBOR_HALF[0] - 0.05
                        and abs(p_map[1] - cy) <= NEIGHBOR_HALF[1] - 0.05):
                    return False
        return (abs(p_map[0] - TABLE_CENTER_XYZ[0]) <= TABLE_HALF_X + margin
                and abs(p_map[1] - TABLE_CENTER_XYZ[1]) <= TABLE_HALF_Y + margin)

    def choose_target(self, targets, exclude=(), check_reach=True, robot_pose=None,
                      on_support_only=True, rank_all=False):
        """返回 (target, obstacles, reason)。exclude 里放已经试过且失败的类别。

        rank_all=True → 返回 (全排序候选 list, [], reason)，供建【抓取清单】用：
        排序键与选最优那个完全一致，所以清单第一项就是原来会选中的目标 ✓


        check_reach：**观察位必须传 False** ✗ —— 观察位离桌 1.15 m，桌上任何物体
        到臂基座都在 1.1~1.3 m，套 [0.10, 0.75] 会把所有候选全拒掉
        （实测：`跳过 sugar_box：距臂基座 1.289 m 不在 [0.10, 0.75]` → Phase 2 直接结束）
        可达性只在【站位上】才有意义 ✓
        """
        cands = []
        for i, t in enumerate(targets):
            cls = t.class_id
            if cls in exclude:
                continue
            # 能不能夹：优先看 objects.yaml 的 graspable，读不到才用兜底名单
            can_grasp = self.graspable.get(cls, cls not in NON_GRASPABLE)
            if not can_grasp:
                if cls not in getattr(self, "_logged_nongrasp", set()):
                    self.log.info("  跳过 {}：objects.yaml 标了 graspable=false（窄边 ≥ 开口）"
                                  .format(cls))
                    self._logged_nongrasp = getattr(self, "_logged_nongrasp", set()) | {cls}
                continue
            # ★ 建清单（rank_all）时不按置信度剔除：清单是"计划"不是"决策" ✗
            #   置信度闸门在每次抓取前照样会过（低置信真目标 17:18 那次就是被它
            #   卡死的），这里先剔掉等于让它连进清单、连被兜底的机会都没有。
            if t.confidence < self.min_confidence and not rank_all:
                self.log.info("  跳过 {}：置信度 {:.2f} < {:.2f}".format(
                    cls, t.confidence, self.min_confidence))
                continue
            # 必须在支撑面（餐桌）上：见 _on_support 的说明
            if on_support_only and robot_pose is not None:
                m = self._to_map((t.point.x, t.point.y), robot_pose)
                if not self._on_support(m):
                    self.log.info("  跳过 {}：不在支撑面（餐桌）上 map({:.3f},{:.3f}) ✗"
                                  .format(cls, m[0], m[1]))
                    continue
            r = math.hypot(t.point.x - ARM_BASE_X, t.point.y)
            if check_reach and not (REACH_MIN <= r <= REACH_MAX):
                self.log.info("  跳过 {}：距臂基座 {:.3f} m 不在 [{:.2f}, {:.2f}]".format(
                    cls, r, REACH_MIN, REACH_MAX))
                continue
            tier = next((k for k, row in enumerate(TARGET_TIERS) if cls in row), len(TARGET_TIERS))
            # 并列时的排序键：离机器人近（x 小）→ 与邻居间隙大 → 置信度高
            gap = min([math.hypot(t.point.x - o.point.x, t.point.y - o.point.y)
                       for j, o in enumerate(targets) if j != i] or [9.9])
            cands.append((tier, t.point.x, -gap, -t.confidence, t, i))
        if not cands:
            return None, [], "没有可夹且可达的目标"
        cands.sort(key=lambda c: c[:4])
        if rank_all:
            return [c[4] for c in cands], [], "全排序 {} 个".format(len(cands))
        _, _, neg_gap, _, best, bi = cands[0]
        obstacles = [t for j, t in enumerate(targets) if j != bi]
        reason = "\"{}\" / 距臂基座 {:.3f} m / 与邻居最近 {:.3f} m / conf {:.2f}".format(
            best.class_id, math.hypot(best.point.x - ARM_BASE_X, best.point.y),
            -neg_gap, best.confidence)
        return best, obstacles, reason

    # ══════════════ ②b 抓取清单（顺序 + 初始位置快照）══════════════
    def _build_plan(self, targets, rp, rp_od):
        """把桌上可夹物体冻结成一份抓取清单：顺序 + 各自 map 初始位置。

        ★ 为什么要冻结（2026-09-18）：原来每轮都重新 fetch_targets + choose_target，
          顺序会跟着当次测量漂（排序键里含 point.x 与邻居间隙）；而且"已放弃"按
          class_id 记 —— 桌上同类摆两个就会互相连坐 ✗。冻结成清单后顺序只定一次，
          放弃按【实例号】记 ✓，且"第一好抓的失败了就直接去下一个"变得显式可控。
        ★ 位置只做兜底：实际站位/下爪仍走"站位重测 + 抓取点重拍"（精度更高），
          清单里的位置只在视觉彻底认不出时才拿来顶一下。
        """
        ranked, _, _ = self.choose_target(targets, check_reach=False,
                                          robot_pose=rp, rank_all=True)
        plan = []
        for i, t in enumerate(ranked):
            plan.append({"idx": i, "class": t.class_id,
                         "xy": self._to_map((t.point.x, t.point.y), rp_od),
                         "conf": float(t.confidence)})
        return plan

    def _rebuild_plan(self, targets, rp=None):
        """近看那一趟看得更全 → 重建一次清单（观察位漏检的物体补进来）。只重建一次 ✓"""
        if self._plan_rebuilt or not targets:
            return
        rp = rp or self._robot_map_pose()
        if rp is None:
            return
        od = self._robot_odom_pose() or rp
        self._plan = self._build_plan(targets, rp, od)
        self._plan_rebuilt = True
        self.log.info("抓取清单已重建：{}".format(self._plan_text(self._plan)))

    @staticmethod
    def _plan_text(plan):
        """清单的日志文本（顺序 + 档位 + 置信度 + 初始 map 位置）。"""
        rows = []
        for p in plan:
            tier = next((k for k, row in enumerate(TARGET_TIERS) if p["class"] in row),
                        len(TARGET_TIERS))
            rows.append("P{} {}(tier{} conf{:.2f} map {:.3f},{:.3f})".format(
                p["idx"] + 1, p["class"], tier, p["conf"], p["xy"][0], p["xy"][1]))
        return " → ".join(rows)

    @staticmethod
    def _next_planned(plan, abandoned):
        """清单里下一个未放弃的条目 → (条目, 实例号)；全试完 → (None, None)。"""
        for p in plan:
            if p["idx"] not in abandoned:
                return p, p["idx"]
        return None, None

    def _plan_xy(self, class_id):
        """清单里该物体的初始 map 位置（认不出时的最后兜底）。"""
        for p in (self._plan or []):
            if p["class"] == class_id:
                return p["xy"]
        return None

    def _pick_fresh(self, targets, planned, rp):
        """清单条目 → 当前视觉里同类的那个检测；认不出返回 None。

        同类有多个时取离【清单初始位置】最近的那个 —— 清单按实例排，而视觉只给
        类别，只能靠位置认回是哪一个。
        """
        same = [t for t in targets if t.class_id == planned["class"]]
        if not same:
            return None
        ox, oy = planned["xy"]       # ★ 清单坐标是 **odom 系**（与 _track/_final_map 同源）
        rod = self._robot_odom_pose() or rp    # ⇒ 比较时也必须用 odom 位姿，混用 map 会错 ✗

        def _d(t):
            m = self._to_map((t.point.x, t.point.y), rod)
            return math.hypot(m[0] - ox, m[1] - oy)

        return min(same, key=_d)

    # ══════════════ ③ 站位解算（目标 map 位置 → 停车点）══════════════
    @staticmethod
    def _to_map(p_base, rp):
        """base_footprint 点 → map 点（rp = 当时的机器人 map 位姿）。"""
        rx, ry, yaw = rp
        c, s = math.cos(yaw), math.sin(yaw)
        return (rx + c * p_base[0] - s * p_base[1],
                ry + s * p_base[0] + c * p_base[1])

    @staticmethod
    def _to_base(p_map, rp):
        """map 点 → 当前 base_footprint 点。"""
        rx, ry, yaw = rp
        dx, dy = p_map[0] - rx, p_map[1] - ry
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * dx + s * dy, -s * dx + c * dy)

    def _mk_target(self, cls, conf, xy_base, stamp=None):
        t = GraspTargetStamped()
        t.header.frame_id = "base_footprint"
        t.header.stamp = stamp if stamp is not None else self.node.get_clock().now().to_msg()
        t.class_id = cls
        t.confidence = float(conf)
        t.point.x, t.point.y, t.point.z = float(xy_base[0]), float(xy_base[1]), TABLE_TOP_Z
        return t

    def _robot_odom_pose(self):
        """odom←base_footprint 的 (x, y, yaw)。

        ★ 最终目标换算必须走 odom，不能走 map ✗：
          map→base_footprint 那一段含 AMCL 误差（实测 0.16 m），而视觉/真值给的是
          **相对量**。用 map 换算等于把 AMCL 误差直接搬进抓取点：
          实测日志里"按 map 算 0.53 m、真值实测 0.98 m"差了 0.45 m，其中 0.16 m 就是它 ✗
          odom 是连续的机器人内部量，短时间尺度上比 AMCL 准得多 ✓
        """
        return self._lookup_pose("odom")

    def _lookup_pose(self, parent_frame):
        if self.tf_buffer is None:
            return None
        for _ in range(TF_RETRY):
            try:
                tr = self.tf_buffer.lookup_transform(parent_frame, "base_footprint", Time())
                t, q = tr.transform.translation, tr.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
                return (t.x, t.y, yaw)
            except Exception:
                self.sleep(0.2)
        return None

    def _robot_map_pose(self):
        if self.tf_buffer is None:
            return None
        for _ in range(TF_RETRY):
            try:
                tr = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
                t, q = tr.transform.translation, tr.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
                return (t.x, t.y, yaw)
            except Exception:
                self.sleep(0.2)
        return None

    def _table_exit_distance(self, obj_map, ax, ay):
        """从物体沿接近方向 (ax, ay) 走到【桌沿外】需要多少米（物体已在桌外则 0）。

        用数值步进而不是解析求交：桌子脚印是轴对齐矩形，方向随便都成立，够用且不会写错。
        """
        step = 0.01
        t = 0.0
        while t <= 1.2:
            x, y = obj_map[0] + ax * t, obj_map[1] + ay * t
            if (abs(x - TABLE_CENTER_XYZ[0]) > TABLE_HALF_X
                    or abs(y - TABLE_CENTER_XYZ[1]) > TABLE_HALF_Y):
                return t
            t += step
        return 1.2

    def creep_goal_for(self, target_map, approach=(0.0, 1.0)):
        """按物体"离桌沿多远"决定站位距离 g（默认 0.33 m）。

        为什么需要自适应：相机装在 z=0.80 m（桌面顶 0.795 m）、且在车心前方 0.076 m。
        物体贴桌沿时（纵深 0.07，旧工作区验证过的情形）0.33 m 站位没问题 ✓；
        但物体放在桌里侧 0.25 m 时，0.33 m 站位会让【相机伸进桌面里 7 mm】✗
        → 相机会撞桌板、creep 永远收敛不了。
        做法：让"相机位置"（车心 + 0.08 m）仍在桌沿之外，所以
            g ≥ 沿接近方向走到桌沿的距离 + 0.08 + 余量 0.05
        纵深 0.07 → g = max(0.33, 0.20) = 0.33 ✓ 旧几何一字不变
        纵深 0.25（sugar_box）→ g = 0.38 ✓
        ★ 接近方向一律传【桌沿法线】（见 approach_normal）：法线是轴对齐的，
          所以"走到桌沿的距离"就是物体到那条边的垂距，数值步进也只是保险。
          旧实现用"机器人当前方位"当方向，方向里的定位误差会让这个距离也跟着错 ✗
        """
        ax, ay = approach
        n = math.hypot(ax, ay)
        if n < 1e-6:
            ax, ay = 0.0, 1.0
        else:
            ax, ay = ax / n, ay / n
        t_exit = self._table_exit_distance(target_map, ax, ay)
        g = max(STANDOFF_DIST, t_exit + CAMERA_FWD + STANDOFF_EDGE_MARGIN)
        if g > STANDOFF_DIST + 1e-3:
            self.log.info("  物体离桌沿 {:.3f} m → 站位从 {:.3f} 拉到 {:.3f} m"
                          "（免得相机撞桌板；相机到桌沿余量 {:.3f} m）"
                          .format(t_exit, STANDOFF_DIST, g, g - t_exit - CAMERA_FWD))
        return g

    def standoff_pose(self, target, standoff=None, robot_map=None, normal=None):
        """由目标的 base_footprint 位置算出 map 里的停车点与朝向。

        ★ 接近方向 = 桌沿法线（approach_normal），不是"机器人当前方位"：
          站位 = 物体 + g × 法线，yaw = 法线反方向 → 车头垂直桌沿、物体在正前方 ✓
        """
        rp = robot_map if robot_map is not None else self._robot_map_pose()
        if rp is None:
            return None
        rx, ry, ryaw = rp
        cy, sy = math.cos(ryaw), math.sin(ryaw)
        bx, by = target.point.x, target.point.y
        ox, oy = rx + cy * bx - sy * by, ry + sy * bx + cy * by       # 目标在 map 里
        ax, ay = normal if normal is not None else self.approach_normal((ox, oy), (rx, ry))
        # 站位必须落在桌子脚印之外（否则目标点落在致命代价上，规划器会拒 ✗）。
        # 用"沿接近方向小步外推"的数值办法，方向随便都成立；
        # 容差取 1e-6：正好等于 EDGE_CLEARANCE 边界时算【外面】（那本来就是留的余量），
        # 这样旧工作区验证过的 (3.0, 2.51) 不会被推到 2.7 ✗
        hx = TABLE_HALF_X + EDGE_CLEARANCE
        hy = TABLE_HALF_Y + EDGE_CLEARANCE

        def inside_table(x, y):
            return (abs(x - TABLE_CENTER_XYZ[0]) < hx - 1e-6
                    and abs(y - TABLE_CENTER_XYZ[1]) < hy - 1e-6)

        t0 = standoff if standoff else STANDOFF_DIST
        t_off = t0
        while t_off < 1.0:
            x, y = ox + ax * t_off, oy + ay * t_off
            if not inside_table(x, y) and self._free(x, y, ROBOT_CLEAR_PARK):
                break
            t_off += 0.01
        else:
            # ★ 外推 1.0 m 仍站不下 → 返回 None，不返回未验证的点 ✗
            #   （2026-09-16：旧实现在这里静默退出循环，把 t_off=1.0 那个
            #     "没查过是否空闲、也远超臂展"的点当站位发给 Nav2：
            #     物体在桌块深处时就会走到这条路径上）
            self.log.warn("  桌沿法线 {} 外推 1.0 m 仍站不下人（物体太深/周围太挤）→ 这一面不可用"
                          .format(tuple(round(v, 3) for v in (ax, ay))))
            return None
        if t_off > t0:
            self.log.info("  站位被桌子/障碍挡住 → 沿桌沿法线外推到 {:.3f} m".format(t_off))
        px, py = ox + ax * t_off, oy + ay * t_off
        yaw = math.atan2(oy - py, ox - px)                             # 面朝物体（= 沿法线指向桌内）
        return (px, py, yaw, ox, oy)

    # ══════════════ ④ 导航 / 对准 / creep / 抓取服务 ══════════════
    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def align_and_approach(self, aim_map, stand_dist=1.15, yaw_tol=0.03,
                           dist_tol=0.15, timeout=25.0):
        """原地对准 map 里的一个点 + 前后微调距离（观察位专用的小闭环）。

        为什么必须有：Nav2 的 xy_goal_tolerance 放宽到 0.4 m（抓取站位靠 creep 兜底），
        观察位没有闭环 → **实测停位偏了 313 mm**，桌子就偏出画面 ✗
        （实测日志：目标观察位 (2.70, 3.20)，实际停在 (2.95, 3.01)）
        这里用 TF 读机器人 map 位姿做"对准 + 靠近"，不查视觉 → 快（几秒）且稳 ✓
        stand_dist 是"机器人到 aim 点"的距离目标；aim 用餐桌中心时 1.15 m 很安全
        （桌面半深 0.25 + 车半径 0.35 = 0.60 m 才会碰到桌沿）。
        """
        moved = False
        deadline = time.time() + timeout
        try:
            while rclpy.ok() and time.time() < deadline:
                rp = self._robot_map_pose()
                if rp is None:
                    self.log.warn("拿不到 map←base_footprint，跳过对准")
                    return False
                rx, ry, ryaw = rp
                dx, dy = aim_map[0] - rx, aim_map[1] - ry
                d = math.hypot(dx, dy)
                err = self._normalize_angle(math.atan2(dy, dx) - ryaw)
                self.log.info("  对准观察点: 距离 {:.3f} m（目标 {:.2f}）偏航 {:+.3f} rad"
                              .format(d, stand_dist, err))
                if abs(err) < yaw_tol and abs(d - stand_dist) < dist_tol:
                    self.log.info("  对准完成 ✓")
                    return True
                if abs(err) > 0.15:                      # 先转正，再进出
                    v, w = 0.0, clamp(1.5 * err, -CREEP_W_MAX, CREEP_W_MAX)
                else:
                    v = clamp(1.2 * (d - stand_dist), -CREEP_V_MAX, CREEP_V_MAX)
                    w = clamp(1.5 * err, -CREEP_W_MAX, CREEP_W_MAX)
                moved = True
                self._publish_cmd_vel(v, w)
                self.sleep(CREEP_DT)
            self.log.warn("对准观察点超时（{:.0f}s）→ 就用当前位姿继续".format(timeout))
            return False
        finally:
            for _ in range(3):
                self._publish_cmd_vel(0.0, 0.0)
                self.sleep(0.2)
            if moved:
                self.sleep(0.4)

    def navigate(self, x, y, yaw):
        for attempt in range(1, MAX_NAV_RETRIES + 1):
            self.log.info("[{}/{}] 导航到 ({:.2f}, {:.2f}, yaw={:.2f})".format(
                attempt, MAX_NAV_RETRIES, x, y, yaw))
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            _, _, qz, qw = yaw_to_quat(yaw)
            goal.pose.pose.orientation.z = qz
            goal.pose.pose.orientation.w = qw
            fut = self.nav_client.send_goal_async(goal)
            if not self.wait_future(fut, GOAL_ACCEPT_TIMEOUT):
                self.log.warn("  目标接受超时，重试")
                continue
            gh = fut.result()
            if gh is None or not gh.accepted:
                self.log.warn("  目标被拒（{}/{}），2s 后重试".format(attempt, MAX_NAV_RETRIES))
                self.sleep(2.0)
                continue
            rf = gh.get_result_async()
            while rclpy.ok() and not rf.done():
                time.sleep(0.1)
            if rf.done() and rf.result().status == 4:
                self.log.info("  ✓ 到达（Nav2 status=4），底盘停止")
                return True
            self.log.warn("  ✗ 导航结束，重试")
            self.sleep(2.0)
        self.log.error("多次导航失败，放弃")
        return False

    def measure(self, class_id, targets_hint=None, refresh_after=600.0):
        """量出某类别物体在 base_footprint 下的 (x, y)；读不到返回 None。

        ★ 快路径：物体是静止的，第一次量到后把它记成 **odom 坐标**，
          之后每轮只用里程计换算到当前底盘系（10 Hz 级，不调视觉）✓
          为什么必须这样：适配层一次调用要 12~15 s（默认+抓取模式+窄词表复核），
          creep 若每轮都调视觉，18 轮 × 13 s = 234 s ✗（预算只有 45 s）
          而 creep 只走 0.2~0.6 m，里程计在这个尺度上漂移几 mm ✓
          全程不碰 gz 真值 ✓（裁判禁止的正是真值）
        ★ refresh_after 原来是 30 s —— 太短了 ✗：现在一次视觉调用就要 25~30 s
          （多帧投票 + 窄词表复核），所以 creep 一开始跟踪基准就"过期" → 转去重新
          调视觉 → 那一帧又没认出这个类别 → creep 直接放弃微调 →
          物体停在 base 0.7 m 外，抓取侧回 BAD_TARGET（距臂基座 0.93 > 0.75）✗
          （实测 2026-09-14 连续 4 轮都卡在这里）。物体在抓取期间不会动，
          600 s 的里程计外推在 0.5 m 尺度上完全够用 ✓
        """
        tr = self._track.get(class_id)
        if tr is not None and (time.time() - tr[0]) <= refresh_after:
            rp_od = self._robot_odom_pose()
            if rp_od is not None:
                xy = self._to_base(tr[1], rp_od)
                self.log.info("  （用里程计跟踪 {}：odom{} → base({:+.3f},{:+.3f})）"
                              .format(class_id, tuple(round(v, 3) for v in tr[1]), xy[0], xy[1]))
                return xy
        if targets_hint is not None:
            for t in targets_hint:
                if t.class_id == class_id:
                    return (t.point.x, t.point.y)
        tg = self.fetch_targets()
        for t in tg:
            if t.class_id == class_id:
                self._remember(class_id, t)
                return (t.point.x, t.point.y)
        # ★ 开集标签是**逐帧抖的**：同一个糖盒这一帧叫 mustard_bottle、下一帧叫
        #   tomato_soup_can（实测 0.35~0.46 的分数、标签一轮一变）→ 按类别名跟踪
        #   会"读不到目标"直接放弃微调 ✗（实测就卡在这里）。
        #   微调只需要【位置】，所以退一步：找"离上次跟踪位置最近的那个检测"，
        #   只要够近（0.25 m）就认它是同一个物体 ✓ 标签原样打日志，不偷偷改类别。
        tr = self._track.get(class_id)
        if tr is not None and tg:
            rp_od = self._robot_odom_pose()
            if rp_od is not None:
                best, best_d = None, 0.25
                for t in tg:
                    q = self._to_map((t.point.x, t.point.y), rp_od)
                    d = math.hypot(q[0] - tr[1][0], q[1] - tr[1][1])
                    if d < best_d:
                        best, best_d = t, d
                if best is not None:
                    self.log.warn("  这次没认出 \"{}\" → 用 {} 的位置代替（相距 {:.3f} m，"
                                  "开集标签会逐帧变，位置才是微调要的量）"
                                  .format(class_id, best.class_id, best_d))
                    self._remember(class_id, best)
                    return (best.point.x, best.point.y)
        return None

    def _remember(self, class_id, tgt):
        """把某类别的当前位置记成 odom 坐标（creep 的跟踪基准）。"""
        rp_od = self._robot_odom_pose()
        if rp_od is None:
            return
        self._track[class_id] = (time.time(), self._to_map((tgt.point.x, tgt.point.y), rp_od))

    def _publish_cmd_vel(self, v, w):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.cmd_pub.publish(m)

    def creep(self, class_id, goal_dist=None):
        """相对闭环微调：把物体开到 base_footprint(0.33, 0) 且方位→0。

        用【相对量】闭环，不查 map → 定位误差与 Nav2 容差都不进这个回路。
        ★ 容差见文件头：方位 0.013 rad ≈ ±4.3 mm，比原来的 0.05 rad 紧得多，
          并且要连续 CREEP_OK_ROUNDS 轮达标才收工。
        """
        goal_d = goal_dist if goal_dist else math.hypot(*CREEP_GOAL_XY)
        self.log.info("相对微调: {} → base_footprint({:.2f}, {:.2f}) ±{:.3f} m / ±{:.3f} rad"
                      .format(class_id, goal_d, CREEP_GOAL_XY[1],
                              CREEP_TOL, CREEP_BEARING_TOL))
        moved, ok_rounds, n = False, 0, 0
        best_err, best_n = None, 0
        deadline = time.time() + CREEP_TIMEOUT
        try:
            while rclpy.ok() and time.time() < deadline:
                xy = self.measure(class_id)
                if xy is None:
                    self.log.warn("微调中读不到目标 → 放弃微调，用当前停位抓取")
                    return False
                d = math.hypot(*xy)
                bearing = math.atan2(xy[1], xy[0])
                dist_err = d - goal_d
                n += 1
                self.log.info("  [{:2d}] 物体 base({:+.3f}, {:+.3f}) 距离 {:.3f} 方位 {:+.3f}"
                              .format(n, xy[0], xy[1], d, bearing))
                if abs(dist_err) < CREEP_TOL and abs(bearing) < CREEP_BEARING_TOL:
                    ok_rounds += 1
                    if ok_rounds >= CREEP_OK_ROUNDS:
                        self.log.info("微调完成（连续 {} 轮达标，共 {} 轮）".format(ok_rounds, n))
                        return True
                    self.sleep(CREEP_DT)
                    continue
                ok_rounds = 0
                # ★ 卡死早退：实测有 152 轮的案例（车被桌子/障碍顶住，命令发出去不动 ✗）
                #   连续 40 轮（≈4 s）距离误差没有改善就收工，别把 45 s 预算烧光
                cur_err = abs(dist_err) + abs(bearing) * 0.33
                if best_err is None or cur_err < best_err - 0.002:
                    best_err, best_n = cur_err, n
                elif n - best_n >= 40:
                    self.log.warn("连续 40 轮没有改善（误差 {:.3f}）→ 判定被顶住，停住底盘用当前停位抓取"
                                  .format(cur_err))
                    return False
                if abs(bearing) > CREEP_SPIN_BEARING:
                    v, w = 0.0, clamp(CREEP_KW * bearing, -CREEP_W_MAX, CREEP_W_MAX)
                else:
                    v = clamp(CREEP_KV * dist_err, -CREEP_V_MAX, CREEP_V_MAX)
                    w = clamp(CREEP_KW * bearing, -CREEP_W_MAX, CREEP_W_MAX)
                moved = True
                self._publish_cmd_vel(v, w)
                self.sleep(CREEP_DT)
            self.log.warn("微调超时（{:.0f}s）→ 停住底盘，用当前停位抓取".format(CREEP_TIMEOUT))
            return False
        finally:
            for _ in range(3):                 # 无论成败都要把速度归零
                self._publish_cmd_vel(0.0, 0.0)
                self.sleep(0.2)
            if moved:
                self.sleep(0.5)                # 等底盘停稳再量/再抓

    def _pose_for_match(self):
        """判"这一帧的检测还是刚才跟的那只物体吗"时用的底盘位姿 —— **优先 AMCL（map 帧）**，
        拿不到 AMCL 才退回里程计。

        ★ 为什么不能用里程计（2026-09-16 实测）：creep 是【里程计】闭环
          （见 measure 的快路径），轮子打滑时命令发出去了、odom 照走、车没动
          → 到抓取点时 odom 已经漂了 0.45 m。用它换算"跟踪位置"，就会把
          【真目标】判成"离跟踪位置超过 0.35 m → 不采信"，再把【邻桌另一只盒子】
          当成顶替（实测那个顶替项与跟踪位置"相距 0.007 m"）→ 抓错物体 ✗
          AMCL 是激光定位、不吃轮子打滑 ✓ 同一帧实测：真 sugar_box 差 0.27 m（采信 ✓）、
          邻桌 gelatin_box 差 0.46 m（拒绝 ✓）—— 判据这下才有分辨力。
        """
        rp = self._robot_map_pose()
        return rp if rp is not None else self._robot_odom_pose()

    def _track_in_map(self, rp_m, tr):
        """把 _track 里的跟踪点换算到 `rp_m` 所在的帧（tr 为 None 时返回 None）。

        ★ 2026-09-18 修 bug：`_remember()` 存的是 `_to_map(p, rp_od)` → `tr[1]` 是
          **odom 系**点；而 `_fresh_target()` 用 `_pose_for_match()`（**优先 AMCL/map**）
          把检测点换算成 **map 系**点，两边帧不一致 —— 直接相减等于把 odom↔map 的
          定位偏移（实测 0.16~0.6 m）当成"物体自己移动了"，于是
          `离跟踪位置超过 0.35 m → 不采信` 恒成立，每次都退回"观察位估计 +
          里程计外推"的降级路径（实跑日志里能看到这条 WARN）。
        `_pose_for_match()` 退回 odom 时本函数自动退化成恒等映射（偏移 0）✓
        做法：两帧只差一个平移 —— 两边 yaw 都以各自原点为零、且是同一底盘朝向，
          用当前两帧的底盘位姿之差补上即可。
        """
        if tr is None or rp_m is None:
            return None
        rp_od = self._robot_odom_pose()
        if rp_od is None:
            return None
        return (tr[1][0] + rp_m[0] - rp_od[0], tr[1][1] + rp_m[1] - rp_od[1])

    def _fresh_target(self, class_id, match_radius=0.35):
        """在【抓取点上】重新量一次目标 → (target, obstacles, source)。

        ★ 为什么必须有这一步（用户 2026-09-14 现场观察：夹爪下去时碰到了盒子顶面）：
          原来最后发给抓取服务的点取自 self._final_map —— 那是【观察位那次】的估计
          （离物体 1 m 以外）再用里程计外推出来的 ✗。中间虽然在站位/爬到位后又拍过照，
          但那些结果只用来挑目标、更新跟踪，**没有更新最终那个点** ✗
          → 手指实际是朝着"1 m 外看出来的位置"下探的，差一两厘米就落到盒子顶面上了
            （盒子只有 38 mm 厚，两指之间的走廊容差只有 ±21 mm）。
        ★ 现在：抓取点上这一帧的检测结果（已经是当前 base_footprint 系、时间戳最新）
          直接就是抓取点 ✓ 不再走 map/odom 往返换算（那一步本身也会引入误差）。
          标签逐帧会抖 → 同名类别优先，找不到就退一步用"离上次跟踪位置最近"的那个。
        """
        tg = self.fetch_targets()
        self.last_targets = tg
        if not tg:
            return None, None, "none"
        same = [t for t in tg if t.class_id == class_id]
        tr = self._track.get(class_id)
        rp_m = self._pose_for_match()
        # ★ 跟踪点是 odom 系的，比较前先换算到 rp_m 所在帧（否则判据恒不成立，
        #   见 _track_in_map 的说明）
        tp = self._track_in_map(rp_m, tr)
        # ★ 同名但位置离谱的检测不要（远处同类别物体 / 误检）：与上次跟踪位置比一比，
        #   超过 match_radius 就当成"不是同一个物体"（爬行期间物体不可能移动 35 cm）
        if same and tr is not None:
            if tp is not None:
                near = []
                for t in same:
                    q = self._to_map((t.point.x, t.point.y), rp_m)
                    if math.hypot(q[0] - tp[0], q[1] - tp[1]) <= match_radius:
                        near.append(t)
                if not near:
                    self.log.warn("  抓取点上认出的 {} 离跟踪位置超过 {:.2f} m → 不采信"
                                  .format(class_id, match_radius))
                same = near
        if not same and tr is not None:
            if tp is not None:
                # ★ 顶替半径比 match_radius 紧得多（0.10 而不是 0.35）：
                #   顶替本来只为救"开集标签逐帧抖"（同一只物体换了名字，位置只差几 mm），
                #   放宽到 0.35 会把【旁边另一只物体】也拉进来 → 抓错东西 ✗
                #   （2026-09-16 实测：顶替进来的 gelatin_box 与跟踪位置"相距 0.007 m"，
                #    而它其实是邻桌 table_2 上的另一只盒子）
                subst_r = min(match_radius, 0.10)
                best, bd = None, subst_r
                for t in tg:
                    q = self._to_map((t.point.x, t.point.y), rp_m)
                    d = math.hypot(q[0] - tp[0], q[1] - tp[1])
                    if d < bd:
                        best, bd = t, d
                if best is not None:
                    self.log.warn("  抓取点上没认出 \"{}\" → 用 {} 的检测代替（相距 {:.3f} m）"
                                  .format(class_id, best.class_id, bd))
                    same = [best]
        if not same:
            return None, None, "none"
        best = max(same, key=lambda t: t.confidence)
        return best, [t for t in tg if t is not best], "vision"

    def _target_suspect(self, tgt, conv_xy):
        """抓取点【可信度】自检 → 返回 None（可信）或一句"为什么不可信"。

        `conv_xy` = 微调收敛后、由里程计跟踪给出的物体位置（base 系），
        必须在 `_remember()` **之前**取 —— `_remember()` 会把跟踪基准覆盖成刚测的
        这个值，取晚了就变成"自己跟自己比"，校验恒成立 ✗

        ★ 为什么要有这一步（2026-09-18 两次实跑）：底盘期间没动，两次测量本该一致。
          失败那次 `_fresh_target()` 给出 base(+0.412,-0.019) conf=0.53，
          而横向偏 19 mm 谁也没拦 —— 旧的安全网是
          `abs(by) > 0.020`（差 1 mm 没触发）且另一路拿 `standoff`（预期距离）当参照，
          而那次站位被外推得更远、预期值从 0.370 漂到 0.400，于是距离那路也没触发
          → 直接照着偏掉的点抓 → 两指合空。
        ★ 本函数不依赖任何"预期值"，只问一句：两次独立测量对得上吗？

        ⚠️ 2026-09-18 四次跑汇总后要留个心眼：横向差把"夹住那次(0 mm)"和"空合那两次
          (19/25 mm)"分开了，但 01:08 那次横向是 **0.000** 却照样空合 ⇒ **横向差不是
          充分判据，过了它也不保证抓得住**。四次跑里真正和成败一一对应的是**伸多远**
          （x≈0.375 夹住；0.411/0.412/0.436 全空合），而伸多远由 `standoff` 决定、
          `standoff = 离桌沿 + 0.130`，`离桌沿` 又跟着 map 系物体估计漂（四次读到
          0.240~0.300，物体其实没动）⇒ 下一轮要把"合爪位指尖的实际 y"记下来
          （见 `call_grasp` 里 tcp_at_max 那段），才能分清是点偏了还是臂没走到位。
        """
        # ★ 置信度【不作为拒抓依据】（2026-09-18 四次实跑定的）——
        #   夹住那次 conf=0.89，可空合的三次里有一次 conf=**0.90**（01:30）、
        #   另两次 0.53。置信度和成败**毫无对应**，拿它当闸门只会误拦：
        #   当前布置下正常检测本来就是 0.52~0.62，0.70 会把每一轮都拦死 ✗
        #   ⇒ 只记录。真正有判别力的是下面的横向差，以及实跑暴露的"伸多远"。
        if tgt.confidence < MIN_GRASP_CONF:
            self.log.warn("  （提示）置信度 {:.2f} < {:.2f} —— 仅记录，不作为拒抓依据"
                          .format(tgt.confidence, MIN_GRASP_CONF))
        if conv_xy is None:
            return None          # 拿不到里程计跟踪值 → 没法交叉验证，不拦（不假装能判）
        dy = abs(tgt.point.y - conv_xy[1])
        if dy > CONSIST_TOL_Y:
            return ("横向差 {:.1f} mm > {:.0f} mm（现场重拍 y={:+.3f} vs 微调收敛 "
                    "y={:+.3f}，底盘期间没动）→ 照这个点下爪两指会合空"
                    .format(dy * 1000, CONSIST_TOL_Y * 1000, tgt.point.y, conv_xy[1]))
        d_new = math.hypot(tgt.point.x, tgt.point.y)
        d_old = math.hypot(conv_xy[0], conv_xy[1])
        if abs(d_new - d_old) > CONSIST_TOL_R:
            return ("距离差 {:.1f} mm > {:.0f} mm（现场重拍 {:.3f} vs 微调收敛 {:.3f}）"
                    .format(abs(d_new - d_old) * 1000, CONSIST_TOL_R * 1000, d_new, d_old))
        return None

    def _nudge(self, class_id, goal_d, rounds=25):
        """按【里程计跟踪】把物体推到 base(goal_d, 0)：补偿刚测出来的残差。

        不再调视觉（一次 25~30 s 太贵），只走 measure() 的 odom 快路径 ✓
        """
        for _ in range(rounds):
            xy = self.measure(class_id)
            if xy is None:
                return False
            d = math.hypot(*xy)
            bearing = math.atan2(xy[1], xy[0])
            if abs(d - goal_d) < CREEP_TOL and abs(bearing) < CREEP_BEARING_TOL:
                return True
            if abs(bearing) > CREEP_SPIN_BEARING:
                v, w = 0.0, clamp(CREEP_KW * bearing, -CREEP_W_MAX, CREEP_W_MAX)
            else:
                v = clamp(CREEP_KV * (d - goal_d), -CREEP_V_MAX, CREEP_V_MAX)
                w = clamp(CREEP_KW * bearing, -CREEP_W_MAX, CREEP_W_MAX)
            self._publish_cmd_vel(v, w)
            self.sleep(CREEP_DT)
        for _ in range(3):
            self._publish_cmd_vel(0.0, 0.0)
            self.sleep(0.2)
        return False

    def call_grasp(self, target, obstacles):
        if not self.wait_client(self.grasp_client, 30.0):
            self.log.error("抓取服务 {} 不可用（grasp_service.launch.py 起了吗？）"
                           .format(GRASP_SERVICE))
            return "unknown", 0, 1, "grasp service unavailable"
        req = GraspFixedObject.Request()
        req.target = target
        req.obstacles = obstacles
        self.log.info("调用 {} : target={} 轴心点({:.3f}, {:.3f}, {:.3f}) 障碍 {} 个".format(
            GRASP_SERVICE, target.class_id, target.point.x, target.point.y, target.point.z,
            len(obstacles)))
        q0 = self._finger_q
        span = None
        sz = self.object_sizes.get(target.class_id)
        if sz:
            span = min(sz[0], sz[1])
            squeeze = self.close_squeeze          # 与 pick_and_place.cpp 同源（都读配置）
            cmd = 0.5 * span - squeeze
            self.log.info("  预期合爪: 物体窄边 {:.4f} m − 干涉 {:.4f} → 关节 {:.4f}"
                          "（两指间距 {:.1f} mm）".format(span, squeeze, cmd, cmd * 2000))
        gap0 = self.finger_gap_mm()
        tcp0 = self.tcp_pose_base()
        # ★ 记轨迹范围（min/max），不能只留"z 最小"的那一次 ✗：
        #   home 位姿的指尖 z(0.914) 比抓取位姿(0.9195) 还低 → 只留最小 z 会永远
        #   记成 home 位姿（实测踩过，害我误判"机械臂没到位" ✗）
        tcp_xs, tcp_zs, tcp_ys = [], [], []
        # ★ 只记 x/z 是**漏掉判别量**的。09-18 四次实跑：x 都执行到位（最远 x ≈ 抓取点 x）、
        #   min z 三次只差 1 mm，可四次里三次空合、一次夹住 ⇒ 差别只可能在没被记录的 y 上。
        #   所以要抓的是【合爪那一瞬指尖到底在哪】——那才是决定"罐子有没有在两指之间"的位姿。
        #     到达位姿 ≈ 命令位姿 → 臂执行没问题，点本身就偏了（该查视觉/定位）
        #     到达位姿 ≢ 命令位姿 → 臂没走到位（该查规划/控制/运动学）
        tcp_at_max = [None]          # 轨迹 x 极值那次（= 抓取位，用于和"抓取点 x"对账）
        tcp_at_close = [None]        # ★ 手指【一开始动】那一瞬的指尖位姿（下爪位的最直接证据）
        if gap0 is not None:
            self.log.info("  合爪前两指真实间距 = {:.1f} mm；指尖平面 base({:+.3f},{:+.3f},{:.3f})"
                          .format(gap0, tcp0[0], tcp0[1], tcp0[2]) if tcp0 else
                          "  合爪前两指真实间距 = {:.1f} mm".format(gap0))
        fut = self.grasp_client.call_async(req)
        qmin, qmax = self._finger_q, self._finger_q          # 夹持过程中采样
        q2min, q2max = self._finger_q2, self._finger_q2      # joint2 单独采（两指独立驱动）
        gmin, gmax = gap0, gap0
        deadline = time.time() + GRASP_SERVICE_TIMEOUT
        while rclpy.ok() and not fut.done() and time.time() < deadline:
            q = self._finger_q
            if q is not None:
                qmin = q if qmin is None else min(qmin, q)
                qmax = q if qmax is None else max(qmax, q)
            q2 = self._finger_q2
            if q2 is not None:
                q2min = q2 if q2min is None else min(q2min, q2)
                q2max = q2 if q2max is None else max(q2max, q2)
            g = self.finger_gap_mm()
            if g is not None:
                gmin = g if gmin is None else min(gmin, g)
                gmax = g if gmax is None else max(gmax, g)
            tp = self.tcp_pose_base()
            if tp is not None:
                tcp_xs.append(tp[0])
                tcp_zs.append(tp[2])
                tcp_ys.append(tp[1])
                if tcp_at_max[0] is None or tp[0] > tcp_at_max[0][0]:
                    tcp_at_max[0] = tp
            # ★ 手指刚开始动 = 合爪那一刻（MTC 是先到位再合爪），把这一瞬的指尖位姿钉住。
            #   用 q0 做基准：它本来就是"合爪前的手指值"，之前赋值了却没用过。
            if (tcp_at_close[0] is None and q0 is not None
                    and self._finger_q is not None and abs(self._finger_q - q0) > 5e-4):
                tcp_at_close[0] = tp if tp is not None else self.tcp_pose_base()
            time.sleep(0.1)
        if not fut.done():
            self.log.error("抓取服务超时（>{:.0f}s）".format(GRASP_SERVICE_TIMEOUT))
            return "unknown", 0, -1, "timeout"
        # ══════════ 合爪判据 ══════════
        # ★ 2026-09-18 改用【TF 实测的两指真实间隙】当主判据，不再用 2×joint1 ✗
        #   mimic 已弃用 → joint2 独立驱动 → 两指**可以不对称**，
        #   "2×joint1" 不再等于真实间隙。实测成功抓取：
        #     joint1 走到指令值 0.0285，joint2 被罐头顶住停在 0.0372，
        #     和 = 0.0657 m ≈ 罐头窄边 0.0660 m ✓  → 两指把罐头夹住了
        #   而旧判据拿 2×0.0285 = 57.0 mm 去比 66.0 mm → 每次成功都误报
        #   "? 物体被挤走" + "✗ 两指不对称"，把真正的失败信号盖掉了 ✗
        if qmin is not None and qmax is not None and abs(qmax - qmin) > 1e-4:
            self.log.info("  合爪实测: joint1 {:.4f}→{:.4f}  joint2 {}（2×joint1 = {:.1f} mm）"
                          .format(qmax, qmin,
                                  "{:.4f}→{:.4f}".format(q2max, q2min) if q2min is not None
                                  else "未采到",
                                  qmin * 2000.0))
        else:
            self.log.warn("  合爪实测: 没采到手指关节变化（/joint_states 没起来？）")
        verdict = "unknown"      # ★ 三态判据：返回给调用方当成败主判据（res.success 会假报空合 ✗）
        if gmin is not None and gmax is not None and abs(gmax - gmin) > 0.5:
            self.log.info("  两指真实间距(TF): 起始 {:.1f} mm → 最小 {:.1f} mm".format(gmax, gmin))
            if span:
                if gmin > (span + 0.004) * 1000.0:
                    verdict = "wide"
                    self.log.warn("  ✗ 手指停在 {:.1f} mm —— 比物体窄边 {:.1f} mm 还宽 "
                                  "→ 没夹到物体（或夹到了别的东西）"
                                  .format(gmin, span * 1000))
                elif gmin > (span - 0.002) * 1000.0:
                    verdict = "gripped"
                    self.log.info("  ✓ 手指停在 {:.1f} mm ≈ 物体窄边 {:.1f} mm "
                                  "→ 夹到了物体（接触即停）".format(gmin, span * 1000))
                else:
                    verdict = "empty"
                    self.log.warn("  ? 手指合到 {:.1f} mm，比物体窄边 {:.1f} mm 还小 "
                                  "→ 物体被挤走 / 没在两指之间".format(gmin, span * 1000))
            # ★ 自检：TF 真实间隙 vs /joint_states 两关节之和，应一致
            #   （不一致 = /joint_states 与运动学对不上，比"两指对不对称"更值得报警）
            if qmin is not None and q2min is not None:
                s = (qmin + q2min) * 1000.0
                if abs(gmin - s) > 8.0:
                    self.log.warn("  ✗ 真实间隙 {:.1f} mm 与 joint1+joint2 = {:.1f} mm 不符 "
                                  "→ /joint_states 与 TF 对不上".format(gmin, s))
                else:
                    self.log.info("  ✓ 与 joint1+joint2 自洽（{:.1f} mm ≈ {:.1f} mm）"
                                  .format(gmin, s))
        elif span:
            self.log.warn("  两指真实间距(TF)没采到 → 无法判定是否夹住 ✗")
        if tcp_xs:
            self.log.info("  指尖轨迹(本次调用): x {:.3f}→{:.3f}（最远 {:.3f}）  z {:.3f}→{:.3f}"
                          "  ← 最远 x 应≈抓取点的 x ✓"
                          .format(min(tcp_xs), max(tcp_xs), max(tcp_xs),
                                  min(tcp_zs), max(tcp_zs)))
            # ★★ 合爪那一瞬的指尖位姿 vs 命令位姿 —— 之前四次实跑缺的就是这一行。
            #   ⚠️ 三个轴不能同等看待：
            #     · x 已有一个约定（"最远 x 应≈抓取点 x"，四次都 ≈ 对上了 ✓）
            #     · **z 有一个固定的上抬**（命令的 z 是"轴心∩支撑面"=桌面 0.780，
            #        指尖实际在桌面上方 ~65 mm 处合爪，四次一致 0.844/0.845）→ 是约定，不是误差
            #     · **y 从来没有基线** —— 而它正是"罐子有没有落在两指之间"的那个轴
            #   ⇒ 先只记录三个差值，跑一次拿到基线；之后 y 的差值才有阈值可言。
            tc = tcp_at_close[0] if tcp_at_close[0] is not None else tcp_at_max[0]
            if tc is not None:
                self.log.info("  合爪位指尖 base({:+.3f},{:+.3f},{:.3f}) vs 命令({:+.3f},{:+.3f},{:.3f})"
                              " → 差 ({:+.1f},{:+.1f},{:+.1f}) mm"
                              .format(tc[0], tc[1], tc[2], target.point.x, target.point.y,
                                      target.point.z, (tc[0] - target.point.x) * 1000,
                                      (tc[1] - target.point.y) * 1000,
                                      (tc[2] - target.point.z) * 1000))
                self.log.info("  本次 y 范围 {:+.3f}~{:+.3f}（合爪位取自{}）"
                              .format(min(tcp_ys), max(tcp_ys),
                                      "手指开始动那一瞬" if tcp_at_close[0] is not None
                                      else "x 极值，没采到动指"))
        res = fut.result()
        if res is None:
            self.log.error("抓取服务无响应")
            return "unknown", 0, -1, "no response"
        self.log.info("抓取服务返回: success={} stage={} [{}] {}".format(
            res.success, res.stage, STAGE_TEXT.get(res.stage, "未知"), res.message))
        return verdict, res.success, res.stage, res.message

    # ══════════════ ⑤ 主流程 ══════════════
    def run(self, observation_pose=None, skip_nav=False, no_creep=False, park_override=None):
        """跑完整个抓取阶段。

        observation_pose = (x, y, yaw)，或 None = 由支撑面自己算（推荐）：
                             桌心沿"机器人这一侧的桌沿法线"外推 OBSERVATION_DIST，
                             朝向面对桌心 → 与站位同一条直线，机械臂全程对正桌子 ✓
        park_override    = (x, y, yaw) 或 None；给了就固定用这个站位（调试用，
                           例如旧工作区那套 (3.0, 2.51, -π/2)）
        """
        self.log.info("=" * 55)
        self.log.info("Phase 2 抓取阶段开始（只用视觉；不读 gz 真值 ✗）")

        if not skip_nav:
            rp0 = self._robot_map_pose()
            if observation_pose is None:
                if rp0 is None:
                    self.log.error("拿不到 map←base_footprint，算不出观察位")
                    return False
                observation_pose = self.observation_pose_for(rp0)
                self.log.info("① 观察位（由桌沿法线算出，与站位同一条法线）")
            else:
                self.log.info("① 先到观察位（能看到整张桌子，便于选目标 + 拿障碍物清单）")
            self.log.info("   观察位 = ({:.3f}, {:.3f}, yaw={:.3f})；桌沿法线 = {}".format(
                observation_pose[0], observation_pose[1], observation_pose[2],
                self.approach_normal(TABLE_CENTER_XYZ[:2],
                                     rp0[:2] if rp0 else (observation_pose[0], observation_pose[1]))))
            if not self.navigate(*observation_pose):
                self.log.warn("观察位不可达 → 直接用站位视觉继续")
            else:
                # ★ 到达后再做一次"对准餐桌中心 + 收到 ~1.15 m"的小闭环：
                #   Nav2 容差 0.4 m 允许停偏 0.3 m（实测偏了 313 mm），桌子会偏出画面 ✗
                self.align_and_approach((TABLE_CENTER_XYZ[0], TABLE_CENTER_XYZ[1]),
                                        stand_dist=OBSERVATION_DIST)
            self.sleep(0.5)

        # ★ 放弃标记按【清单实例号】记：按 class_id 记会让同类物体互相连坐
        #   （桌上摆两个 coke can 时，放弃第一个会把第二个一起排掉 ✗）
        abandoned = set()
        close_look_done = False
        for _attempt in range(1, MAX_TARGET_TRIES + 1):
            targets = self.fetch_targets()
            self.last_targets = targets
            # ★ 一个都没返回时，也要试一次"近看"（实测踩过）：
            #   小物体（汤罐 66×101 mm）在观察位 1.15 m 处视觉完全认不出来 ✗，
            #   而原来的近看回退只在"有候选但都不合格"时才触发 → 直接结束 ✗
            if not targets and not skip_nav and not close_look_done:
                close_look_done = True
                self.log.warn("观察位没返回任何目标 → 靠近到近看位再试一次")
                rp_c = self._robot_map_pose()
                if rp_c is not None:
                    n = self.approach_normal(TABLE_CENTER_XYZ[:2], rp_c[:2])
                    cx = TABLE_CENTER_XYZ[0] + n[0] * CLOSE_LOOK_DIST
                    cy = TABLE_CENTER_XYZ[1] + n[1] * CLOSE_LOOK_DIST
                    cyaw = math.atan2(TABLE_CENTER_XYZ[1] - cy, TABLE_CENTER_XYZ[0] - cx)
                    if self.navigate(cx, cy, cyaw):
                        self.align_and_approach((TABLE_CENTER_XYZ[0], TABLE_CENTER_XYZ[1]),
                                                stand_dist=CLOSE_LOOK_DIST, timeout=15.0)
                        self.sleep(0.5)
                        targets = self.fetch_targets()
                        self.last_targets = targets
                        self.log.info("近看返回 {} 个目标".format(len(targets)))
                        # ★ 近看位看得更清 → 重建一次清单（观察位漏检的物体补进来）
                        self._rebuild_plan(targets)
            if not targets and self._plan is None:
                self.log.error("视觉没给出任何目标（观察位 + 近看位都没有）→ 结束抓取阶段（不会退真值 ✗）")
                return False
            if not targets:
                # ★ 清单已经定下来了 → 即使这一步视觉全空也按清单继续，
                #   位置退回初始快照（这正是"记住四个初始位置"的用处之一 ✓）
                self.log.warn("这一步视觉没给出任何目标 → 按手里的清单继续（位置用初始快照）")
            self.log.info("候选目标 {} 个（来源 vision）：{}".format(
                len(targets),
                ["{}({:.2f})".format(t.class_id, t.confidence) for t in targets]))
            # 选目标前先拿机器人的 map 位姿：①支撑面校验 ②站位解算 都要用
            rp = self._robot_map_pose()
            if rp is None:
                self.log.error("拿不到 map←base_footprint 变换，算不出站位")
                return False
            # ★ 建清单：观察位第一次拿到候选时定一次（顺序 + 初始位置快照）
            if self._plan is None and targets:
                od = self._robot_odom_pose() or rp
                self._plan = self._build_plan(targets, rp, od)
                self.log.info("抓取清单（按好抓程度排序）：{}".format(self._plan_text(self._plan)))
            # ★ 清单仍为空（候选全被可夹性/支撑面过滤掉）→ 借近看位重建一次
            if not self._plan and not skip_nav and not self._plan_rebuilt:
                self.log.warn("观察位挑不出可夹物体 → 靠近到近看位重建清单")
                n = self.approach_normal(TABLE_CENTER_XYZ[:2], rp[:2])
                cx = TABLE_CENTER_XYZ[0] + n[0] * CLOSE_LOOK_DIST
                cy = TABLE_CENTER_XYZ[1] + n[1] * CLOSE_LOOK_DIST
                cyaw = math.atan2(TABLE_CENTER_XYZ[1] - cy, TABLE_CENTER_XYZ[0] - cx)
                if self.navigate(cx, cy, cyaw):
                    self.align_and_approach((TABLE_CENTER_XYZ[0], TABLE_CENTER_XYZ[1]),
                                            stand_dist=CLOSE_LOOK_DIST, timeout=15.0)
                    self.sleep(0.5)
                    rp = self._robot_map_pose() or rp
                    targets = self.fetch_targets()
                    self.last_targets = targets
                    self.log.info("近看候选 {} 个：{}".format(
                        len(targets),
                        ["{}({:.2f})".format(t.class_id, t.confidence) for t in targets]))
                    self._rebuild_plan(targets, rp)
                else:
                    self.log.warn("近看位不可达")
            if not self._plan:
                self.log.error("没有可夹的目标（候选为空或全被过滤）→ 结束抓取阶段")
                return False
            # ★ 从清单取下一个未放弃的物体：顺序开局就冻结好了，不再每轮重选 ✓
            planned, pidx = self._next_planned(self._plan, abandoned)
            if planned is None:
                self.log.error("清单里的 {} 个目标都试过了 → 结束抓取阶段".format(len(self._plan)))
                return False
            target = self._pick_fresh(targets, planned, rp)
            if target is not None:
                obstacles = [t for t in targets if t is not target]
                reason = "视觉 base({:+.3f},{:+.3f}) conf={:.2f}".format(
                    target.point.x, target.point.y, target.confidence)
            else:
                # ★ 这一步视觉没认出它 → 退回清单里的初始位置（精度差，故明确告警）
                #   清单坐标是 odom 系 ⇒ 换算回车体必须配 odom 位姿，配 rp(map) 会错 ✗
                obstacles = list(targets)
                rod = self._robot_odom_pose() or rp
                target = self._mk_target(planned["class"], planned["conf"],
                                         self._to_base(planned["xy"], rod))
                reason = "清单初始位置 map({:.3f},{:.3f})".format(
                    planned["xy"][0], planned["xy"][1])
                self.log.warn("这一步没认出 {} → 退回清单初始位置".format(planned["class"]))
            self.log.info("→ 选中 [{}]（清单 P{}）：{}".format(
                target.class_id, pidx + 1, reason))

            cy, sy = math.cos(rp[2]), math.sin(rp[2])
            obj_map = (rp[0] + cy * target.point.x - sy * target.point.y,
                       rp[1] + sy * target.point.x + cy * target.point.y)
            # ★ 接近方向 = 桌沿法线（不是"物体→机器人"那种含定位误差的方向）
            normal = self.approach_normal(obj_map, rp[:2], check_reach=True)
            standoff = self.creep_goal_for(obj_map, normal)
            self.log.info("  桌沿法线 = ({:+.3f}, {:+.3f}) → 机械臂正对桌子（yaw = {:.3f}）"
                          .format(normal[0], normal[1], math.atan2(-normal[1], -normal[0])))

            # ★ 目标与障碍一律记【map 坐标】：底盘之后还会动（导航+creep），
            #   观察位量到的 base 系点到了站位就是错的 ✗
            #   （实测：站位上视觉没认出 sugar_box → 沿用了观察位那次 base(1.184,0.249)
            #    的相对量 → 手被指到 1.18 m 外，根本不是物体在的地方）
            # 用 odom 记账（不是 map ✗）：见 _robot_odom_pose 的说明
            rp_od = self._robot_odom_pose() or rp
            self._track[target.class_id] = (time.time(),
                                            self._to_map((target.point.x, target.point.y), rp_od))
            self._final_map = {"class": target.class_id, "conf": target.confidence,
                               "xy": self._to_map((target.point.x, target.point.y), rp_od)}
            self._final_obs = [{"class": o.class_id, "conf": o.confidence,
                                "xy": self._to_map((o.point.x, o.point.y), rp_od)}
                               for o in obstacles]

            # 站位：优先用命令行覆盖（调试），否则由目标位置算
            if park_override is not None:
                px, py, pyaw = park_override
                _, _, _, ox, oy = (self.standoff_pose(target, standoff, rp, normal)
                                   or (px, py, pyaw, px, py))
                self.log.info("站位被命令行固定为 ({:.3f}, {:.3f}, yaw={:.3f})（调试）"
                              .format(px, py, pyaw))
            else:
                pose = self.standoff_pose(target, standoff, rp, normal)
                if pose is None:
                    # None 有两个来源：拿不到位姿变换，或这一面外推 1.0 m 仍站不下人
                    # （后者见 standoff_pose 的说明）——都判"这个物体不再抓"，去下一个
                    abandoned.add(pidx)
                    self.log.error("算不出可用站位（位姿变换缺失，或这一面站不下人）"
                                   "→ 判定 {} 不再抓，换下一个".format(target.class_id))
                    continue
                px, py, pyaw, ox, oy = pose
            self.log.info("② 抓取站位 = ({:.3f}, {:.3f}, yaw={:.3f})；目标在 map({:.3f}, {:.3f})"
                          .format(px, py, pyaw, ox, oy))
            if not skip_nav and not self.navigate(px, py, pyaw):
                self.log.error("站位不可达 → 结束")
                return False
            self.sleep(1.0)

            # 站位上重测（相机与桌面等高，站位上物体整只可见 ✓）
            self.sleep(0.5)
            targets = self.fetch_targets()
            self.last_targets = targets
            t2, obs2, _ = self.choose_target(targets, exclude=(), check_reach=True,
                                             robot_pose=self._robot_map_pose())
            if t2 is not None and t2.class_id == target.class_id:
                target, obstacles = t2, obs2
            else:
                self.log.warn("站位上没重新检测到 {} → 沿用观察位那次的量".format(target.class_id))

            if not no_creep:
                self.creep(target.class_id, standoff)

            # ══════════ ④ 抓取点上的【最后一张照片】= 最终抓取点 ══════════
            # （用户在 Gazebo 里看到"夹爪下去碰到盒子顶面"就是这里原来用的点太旧 ✗）
            best, obs, src = self._fresh_target(target.class_id)
            if best is not None:
                # ★ 必须重打时间戳：适配层那条消息带的是【拍照时刻】的 stamp，
                #   而它一帧要 12~35 s（3 帧投票 + 窄词表复核）⇒ 直接透传会被抓取侧
                #   判"目标不新鲜 age=12.9s > 10s"拒收 ✗（实测 stage=1 NO_TARGET）
                #   位置还是那次测量的位置（刚测完，底盘没动），时间戳用"此刻" ✓
                now = self.node.get_clock().now().to_msg()
                best.header.stamp = now
                for o in obs:
                    o.header.stamp = now
                # ★★ 自检的参照值必须在 `_remember()` **之前**取 ——
                #    `_remember()` 会把跟踪基准覆盖成 best 自己，取晚了就成了
                #    "自己跟自己比"，校验恒成立 ✗（底盘此刻没动，所以这个值就是
                #    微调收敛值，是独立于这张照片的另一次测量）
                conv_xy = self.measure(target.class_id)
                ok_point = False
                for _try in range(CONSIST_TRIES + 1):
                    why = self._target_suspect(best, conv_xy)
                    if why is None:
                        if _try:
                            self.log.info("  ✓ 重拍 {} 次后自检通过".format(_try))
                        ok_point = True
                        break
                    self.log.warn("  ✗ 抓取点自检不过（第 {} 次）：{}".format(_try + 1, why))
                    if _try >= CONSIST_TRIES:
                        break
                    self.sleep(0.5)        # 只为区分"偶发"和"稳定"（重拍救不回来，见 CONSIST_TRIES 注释）
                    b2, o2, _s2 = self._fresh_target(target.class_id)
                    if b2 is None:
                        break
                    now_r = self.node.get_clock().now().to_msg()
                    b2.header.stamp = now_r
                    for o in o2:
                        o.header.stamp = now_r
                    best, obs = b2, o2
                    self.log.info("  重拍: {} base({:+.3f},{:+.3f}) conf={:.2f}"
                                  .format(best.class_id, best.point.x, best.point.y,
                                          best.confidence))
                if not ok_point:
                    # ★ 2026-09-18：先**降级为告警，照抓**，理由有三 ——
                    #   ① 拒抓等于这一轮颗粒无收，可"只求能成功抓准"要的是能抓上；
                    #   ② 判断"点偏没偏"真正缺的那个数是【合爪位指尖的实际 y】，
                    #      而它是在 call_grasp 里打的 —— 这里 return 掉就永远拿不到 ✗；
                    #   ③ 三次空合都证明对罐子无害（合完罐子原地没动），
                    #      而 sugar_box 那次被推翻是**另一个物性**（高瘦盒），不是这条路径。
                    #   ⇒ 拿到足够数据、定出真正的闸门（现在看是"伸多远"）之后再决定要不要恢复。
                    self.log.error("  ✗✗ 抓取点自检连续 {} 次不过 → 【仍照抓，仅告警】"
                                   "（拒抓已降级；本轮的价值是拿到合爪位指尖 y 的实测）"
                                   .format(CONSIST_TRIES + 1))
                self._remember(target.class_id, best)      # 同时刷新里程计跟踪
                bx, by = best.point.x, best.point.y
                self.log.info("抓取点重测: {} base({:+.3f},{:+.3f}) conf={:.2f}（障碍 {} 个，来源 {}）"
                              .format(best.class_id, bx, by, best.confidence, len(obs), src))
                # 残差偏大 → 按同一套相对控制定量补一次（纯里程计），再拍一张确认
                # ★ 2026-09-18：门限从写死的 0.020 改成自检那个常量 CONSIST_TOL_Y(0.012)。
                #   `_target_suspect` 的 docstring 早就指出"旧的安全网 abs(by)>0.020 差 1 mm
                #   没触发"是空合的一环；今天又拿到两例：01:28 / 01:55 的视觉横向都是 −0.019，
                #   刚好卡在 0.020 之下 ⇒ 补正根本没触发 ⇒ 19 mm 直接下爪 ⇒ 空合。
                #   两个门限共用同一常量，以后不会再各走各的 ✓
                if (abs(by) > CONSIST_TOL_Y or abs(math.hypot(bx, by) - standoff) > 0.030):
                    self.log.warn("  残差偏大（横向 {:+.3f} m / 距离 {:.3f} vs {:.3f}）→ 定量补一次"
                                  .format(by, math.hypot(bx, by), standoff))
                    self._nudge(target.class_id, standoff)
                    self.sleep(0.5)
                    best2, obs2, src2 = self._fresh_target(target.class_id)
                    if best2 is not None:
                        # ★ 同样要重打时间戳（第一次忘了会导致第二次测量被"不新鲜"拒收 ✗）
                        now2 = self.node.get_clock().now().to_msg()
                        best2.header.stamp = now2
                        for o in obs2:
                            o.header.stamp = now2
                        # ★ 参照值同样要在 _remember 之前取（理由见上面 conv_xy 处）
                        conv2 = self.measure(target.class_id)
                        self._remember(target.class_id, best2)
                        best, obs = best2, obs2
                        self.log.info("  补正后重测: {} base({:+.3f},{:+.3f}) conf={:.2f}"
                                      .format(best.class_id, best.point.x, best.point.y,
                                              best.confidence))
                        # ★ 同一个自检也要过一遍：**这个值才是最终下爪用的那个**。
                        #   参照换成"补正后"的里程计跟踪值（底盘刚动过，不能再用 conv_xy）
                        #   —— 若横向对不上，说明这次补正没把它推到位。
                        #   处理同上面：**只告警，照抓**（拒抓已降级，理由见上）
                        why2 = self._target_suspect(best, conv2)
                        if why2 is not None:
                            self.log.error("  ✗✗ 补正后抓取点自检不过：{} → 【仍照抓，仅告警】"
                                           .format(why2))
                    else:
                        # ★★ 2026-09-18 现场修：补正**已经真把底盘挪了** ⇒ 绝不能再拿补正前的
                        #   旧点下爪。02:32 空合的直接原因就在这里：`_nudge` 把底盘横向挪了
                        #   ~16 mm（跟踪值 −0.021→−0.005），重拍又失败，代码于是继续用补正前的
                        #   (0.442,-0.021) —— 那是**旧车体系**里的点，在新车体系里偏 16 mm，
                        #   超过 ±5 mm 捕获窗口 ⇒ 空合。（10:05 能夹住，唯一差别就是这次重拍
                        #   成功了、拿到补正后的 −0.004。）
                        #   重拍失败时的退路：**里程计跟踪值**（measure 的快路径，不调视觉）。
                        #   它的锚点在上面刚被 `_remember` 设成那张照片的值，之后只走了很短
                        #   一段相对运动 ⇒ 是"视觉锚点 + 小相对量"，这段的相对误差只有几 mm
                        #   （实测对照：10:05 补正后 跟踪 0.000 vs 重拍 −0.004 ⇒ 差 4 mm）
                        tr = self.measure(target.class_id)
                        if tr is None:
                            # ★ 快照兜底（顺序：里程计外推 → 清单初始位置）：里程计锚点也没了
                            #   → 退回清单里的 map 初始位置。底盘此刻【已经动过】（_nudge 挪的）
                            #   ⇒ 必须用【当前】map 位姿重新换算，绝不能用补正前的旧车体系点 ✗
                            pxy = self._plan_xy(target.class_id)
                            rp3 = self._robot_odom_pose()   # 清单是 odom 系 → 必须配 odom ✗
                            if pxy is None or rp3 is None:
                                self.log.error("  ✗✗ 补正后重拍失败、里程计跟踪也拿不到、清单里也没有"
                                               "位置 → 判定 {} 不再抓（绝不拿补正前的旧点下爪）"
                                               .format(target.class_id))
                                abandoned.add(pidx)
                                continue
                            self.log.warn("  补正后重拍失败、里程计也拿不到 → 退回清单初始位置"
                                          " map({:.3f},{:.3f})（精度差，可能碰物体）"
                                          .format(pxy[0], pxy[1]))
                            best = self._mk_target(target.class_id, target.confidence,
                                                   self._to_base(pxy, rp3))
                            best.header.stamp = self.node.get_clock().now().to_msg()
                        else:
                            dx, dy = float(tr[0]) - bx, float(tr[1]) - by  # 底盘位移在车体系里的表现
                            best.point.x, best.point.y = float(tr[0]), float(tr[1])
                            now3 = self.node.get_clock().now().to_msg()
                            best.header.stamp = now3    # 底盘刚动过，时间戳必须刷新
                            for o in obs:               # 障碍也是补正前量的 ⇒ 按同一位移一起搬
                                o.point.x += dx
                                o.point.y += dy
                                o.header.stamp = now3
                            self._remember(target.class_id, best)
                            self.log.warn("  补正后重拍失败 → 改用里程计跟踪点 base({:+.3f},{:+.3f})"
                                          "（相对补正前 {:+.0f}/{:+.0f} mm；无独立测量，故不跑自检）"
                                          .format(best.point.x, best.point.y,
                                                  dx * 1000.0, dy * 1000.0))
                target, obstacles = best, obs
            else:
                # 抓取点上认不出来 → 退回"观察位估计 + 里程计外推"，并明确告警
                self.log.warn("抓取点上认不出 {} → 退回观察位估计 + 里程计外推（精度差，可能碰物体）"
                              .format(target.class_id))
                rp2 = self._robot_odom_pose() or self._robot_map_pose()
                if rp2 is None:
                    self.log.error("拿不到任何底盘位姿，没法把目标换算到当前底盘系")
                    return False
                fm = getattr(self, "_final_map", None)
                if fm is None:
                    # ★ 快照兜底：连"观察位那次"的锚点都没有时，用清单里的初始位置 ✓
                    pxy = self._plan_xy(target.class_id)
                    if pxy is None:
                        self.log.error("  ✗✗ 既没有测量锚点、清单里也没有 {} → 判定不再抓，换下一个"
                                       .format(target.class_id))
                        abandoned.add(pidx)
                        continue
                    self.log.warn("  退回清单初始位置 map({:.3f},{:.3f})".format(pxy[0], pxy[1]))
                    fm = {"class": target.class_id, "conf": target.confidence, "xy": pxy}
                target = self._mk_target(fm["class"], fm["conf"], self._to_base(fm["xy"], rp2))
                obstacles = [self._mk_target(o["class"], o["conf"], self._to_base(o["xy"], rp2))
                             for o in getattr(self, "_final_obs", [])]

            verdict, ok, stage, msg = self.call_grasp(target, obstacles)
            # ★ 成败判据 = TF 两指间隙（不是 res.success ✗ —— MTC 空合也报 success=True，
            #   实测 57mm 空合照样走 "if ok" 假报成功）。间隙采不到(unknown)才退回 res.success。
            gripped = (verdict == "gripped") or (verdict == "unknown" and ok)
            if gripped:
                self.log.info("Phase 2 完成：抓取成功 ✓（TF 间隙判据={}）".format(verdict))
                return True
            # ★ 环境级故障（3 SCENE_FAILED / 4 INIT_FAILED）：换目标也是同样的错，
            #   四个挨个撞一遍只是把同一份错误日志打四遍、白耗时间 → 直接结束 ✓
            if stage in (3, 4):
                self.log.error("抓取失败（stage={} [{}]，环境级故障）→ 结束抓取阶段"
                               .format(stage, STAGE_TEXT.get(stage, "未知")))
                return False
            if stage == 0 and verdict in ("empty", "wide"):
                # ★ 重新拍照识别重新抓：MTC 报告成功(stage=0)但两指没夹到（空合/没合到）。
                #   纯原地重拍救不回来（视觉是确定性的，同一位置重拍返回逐位相同的结果 ✗），
                #   所以先 _nudge 挪一点底盘换测量，再 _fresh_target 重拍重识别、重新下爪。
                for _retry in range(1, SAME_TARGET_RETRY + 1):
                    self.log.warn("两指没夹到（verdict={}）→ 第 {} 次重新拍照识别重新抓"
                                  .format(verdict, _retry))
                    self._nudge(target.class_id, standoff)
                    self.sleep(0.5)
                    best, obs, _s = self._fresh_target(target.class_id)
                    if best is None:
                        self.log.warn("重拍没认出 {} → 放弃重抓".format(target.class_id))
                        break
                    now = self.node.get_clock().now().to_msg()
                    best.header.stamp = now
                    for o in obs:
                        o.header.stamp = now
                    self._remember(target.class_id, best)
                    target, obstacles = best, obs
                    verdict, ok, stage, msg = self.call_grasp(target, obstacles)
                    if verdict == "gripped" or (verdict == "unknown" and ok):
                        self.log.info("Phase 2 完成：抓取成功 ✓（重抓第 {} 次，TF 间隙判据={}）"
                                      .format(_retry, verdict))
                        return True
                    if stage in (3, 4):
                        self.log.error("重抓第 {} 次遇到环境级故障（stage={} [{}]）→ 结束抓取阶段"
                                       .format(_retry, stage, STAGE_TEXT.get(stage, "未知")))
                        return False
                    if stage != 0:
                        self.log.warn("重抓第 {} 次失败（stage={} [{}]）→ 换目标".format(
                            _retry, stage, STAGE_TEXT.get(stage, "未知")))
                        break
                    self.log.warn("重抓第 {} 次仍没夹到（verdict={} stage={}）".format(
                        _retry, verdict, STAGE_TEXT.get(stage, stage)))
                # ★ 重抓耗尽仍未成功 → 判定这个物体不再抓，去下一个 ✓
                #   （原来这里直接 return False、整个 Phase 2 收场 —— 等于第一好抓的物体
                #     一次没成就放弃全部，与"按好抓程度排序 + 依次尝试"的意图相反 ✗）
                abandoned.add(pidx)
                self.log.warn("{} 试满 {} 次重抓仍未成功（stage={} [{}]）→ 判定不再抓，换下一个"
                              .format(target.class_id, SAME_TARGET_RETRY, stage,
                                      STAGE_TEXT.get(stage, "未知")))
                continue
            # ★ 目标相关类失败（1 NO_TARGET / 2 BAD_TARGET / 5 NO_SOLUTION / 6 EXEC_FAILED）
            #   → 判定不再抓，换下一个物体
            abandoned.add(pidx)
            self.log.warn("目标 {} 失败（stage={} [{}]）→ 判定不再抓，换下一个"
                          .format(target.class_id, stage, STAGE_TEXT.get(stage, "未知")))
            continue
        self.log.error("清单里的目标都试过仍未成功（共 {} 个）→ 结束抓取阶段"
                       .format(MAX_TARGET_TRIES))
        return False

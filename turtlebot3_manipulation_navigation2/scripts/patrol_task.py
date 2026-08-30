#!/usr/bin/env python3
"""
任务节点 — 初始定位 → 巡逻识别 → 汇总输出

执行流程:
  1. 发布初始位姿到 /initialpose
  2. 等待 Nav2 就绪
  3. 依次导航到 4 个观察点（到达 → GroundingDINO+SAM2 识别 → 下一个）
  4. 汇总输出每个观察点的检测结果（目标数量 + 像素中心点）

用法（需先激活 /home/jasmine/vision_env，含 torch/groundingdino/sam2 与 rclpy）:
  source /home/jasmine/vision_env/bin/activate
  ros2 run turtlebot3_manipulation_navigation2 patrol_task.py
"""

import math
import os
import threading
import time

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
import tf2_ros

# 视觉识别流水线（GroundingDINO + SAM2），单独文件便于复用 / 离线测试
import vision_pipeline


# ═══════════════════════════════════════════════════════════════
# 配置参数
# ═══════════════════════════════════════════════════════════════

# 注意：map 帧与 Gazebo world 帧重合（map.yaml 的 origin
# [-5.62, -4.35] 只是把地图图像对齐到世界，并非坐标偏移）。
# 因此下面的世界坐标可直接作为 map 帧坐标发布，无需转换。

# 初始位姿 — 和 turtlebot3_franka.launch.py spawn 参数一致
INITIAL_X = -5.30
INITIAL_Y = -0.50
INITIAL_YAW = 0.0

# 客厅 4 张桌子的观察点 (名称, x, y, yaw)
# 距离桌面约 1.2m，相机面向桌子长边
# 访问顺序按 Gazebo 实际布局：table_3 → table_1 → table_0 → table_2
WAYPOINTS = [
    ("table_3", -4.95, -3.1, math.pi / 50),           # 面朝 +X（living_room_table_3）
    ("table_1", -3.3, -1.2, - math.pi / 2), # 面朝 -Y（living_room_table_1）
    ("table_0", -2.1, -1.2, math.pi*7 / 12),   # 面朝 +Y（living_room_table_0）
    ("table_2", -0.48, -2.6, math.pi / 25),           # 面朝 +X（living_room_table_2）
]

# 相机 RGB 话题：Gazebo 桥接把 /pi_camera/image 重映射成了 /camera/image_raw，
# 所以实际订阅 /camera/image_raw。真机 / 其他相机请改成你的话题名。
CAMERA_IMAGE_TOPIC = "/camera/image_raw"

# 深度图 + 相机内参（rgbd_camera 的 RGB 与深度对齐，反投影用）
DEPTH_IMAGE_TOPIC = "/camera/depth/image_raw"
CAMERA_INFO_TOPIC = "/camera/camera_info"

# RViz Marker 话题与各物品颜色
MARKER_TOPIC = "/detected_items"
ITEM_COLORS = {
    "apple": (1.0, 0.0, 0.0),      # 红
    "coke can": (0.0, 0.4, 1.0),   # 蓝
    "bowl": (0.0, 1.0, 0.0),       # 绿
    "banana": (1.0, 1.0, 0.0),     # 黄
}

# 同一物理物品的 /map 坐标去重阈值（米）。不同物品在同一桌上相距 ≥0.3m，
# 而同一物品从不同观察点看到的坐标误差 <0.05m，取 0.25 安全区分两者。
DEDUP_DIST = 0.25

# 物品最小高度（米，/map 帧 z）。桌面约 0.78m，桌上物品中心 z≈0.81~0.84；
# 桌腿/地面等结构被误检时 z≈0.60。低于此值判为「不是桌上物品」直接丢弃。
MIN_OBJECT_Z = 0.70

# 是否保存每个观察点的标注结果图（检测框 + 分割轮廓 + 中心点），调试用
SAVE_DEBUG_IMAGE = True
DEBUG_IMAGE_DIR = os.path.expanduser("~/turtlebot3_detections")


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class CompetitionTask(Node):
    def __init__(self):
        super().__init__("patrol_task")

        # 初始位姿发布
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        # Nav2 action 客户端
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        self.waypoint_idx = 0
        self.detection_results = {}

        # 目标被拒时的重试状态（Nav2 的 bt_navigator 要等 costmap 激活
        # 完成后才真正接受目标，server_is_ready() 会过早返回 True）
        self.goal_retry_count = 0
        self.goal_retry_timer = None
        self.MAX_GOAL_RETRIES = 8
        self.GOAL_RETRY_DELAY = 2.0

        # 相机图像订阅：只缓存最新一帧，到观察点后再取出来识别
        self._latest_frame = None
        self._frame_lock = threading.Lock()
        self.create_subscription(
            Image, CAMERA_IMAGE_TOPIC, self._on_image_cb, 10
        )

        # 深度图 + 相机内参（像素→3D→map 反投影用）
        self._latest_depth = None
        self._depth_lock = threading.Lock()
        self.create_subscription(
            Image, DEPTH_IMAGE_TOPIC, self._on_depth_cb, 10
        )
        self._camera_info = None
        self._info_lock = threading.Lock()
        self.create_subscription(
            CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info_cb, 10
        )

        # TF（相机光学帧 → map）与 RViz Marker 发布
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._camera_frame = None          # 延迟解析真实光学帧名
        self._marker_pub = self.create_publisher(Marker, MARKER_TOPIC, 10)
        self._next_marker_id = 0
        self._seen_objects = []            # 已登记物品（/map 位置 + 深度，用于去重）
        self._item_counts = {name: 0 for name in vision_pipeline.ITEM_NAMES}

        # 视觉流水线延迟到首次识别时才加载（模型加载耗时 + 占显存）
        self._pipeline = None
        self._pipeline_error = None

        self.get_logger().info("CompetitionTask 节点已就绪。")

    # ── 入口 ───────────────────────────────────────────────────

    def start(self):
        self.get_logger().info("=" * 45)
        self.get_logger().info("任务启动: 初始定位 → {} 个观察点巡逻".format(len(WAYPOINTS)))
        self.get_logger().info("=" * 45)
        # 先等 Nav2 就绪，再发初始位姿（否则 AMCL 还没启动，收不到 /initialpose）
        self._wait_for_nav2()
        self._publish_initial_pose()
        self._navigate_next()

    # ── 初始定位 ───────────────────────────────────────────────

    def _publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = INITIAL_X
        msg.pose.pose.position.y = INITIAL_Y
        _, _, qz, qw = yaw_to_quat(INITIAL_YAW)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        for _ in range(5):
            self.init_pub.publish(msg)
            time.sleep(0.2)

        self.get_logger().info(
            "初始位姿: ({:.2f}, {:.2f}, yaw={:.2f})".format(INITIAL_X, INITIAL_Y, INITIAL_YAW)
        )

    def _wait_for_nav2(self):
        print("等待 Nav2 action server...", flush=True)
        self.get_logger().info("等待 Nav2 action server...")
        timeout = time.time() + 60.0
        while rclpy.ok() and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.nav_client.server_is_ready():
                print("Nav2 已就绪。", flush=True)
                self.get_logger().info("Nav2 已就绪。")
                return True
            time.sleep(0.5)
        print("Nav2 超时！请确认 navigation.launch 已启动。", flush=True)
        self.get_logger().error("Nav2 action server 超时！")
        time.sleep(0.2)  # 给 rosout 刷出日志
        raise RuntimeError("Nav2 action server not available")

    # ── 串行导航 ───────────────────────────────────────────────

    def _navigate_next(self):
        if self.waypoint_idx >= len(WAYPOINTS):
            self._all_done()
            return

        name, x, y, yaw = WAYPOINTS[self.waypoint_idx]
        self.get_logger().info(
            "[{}/{}] 导航至 {} ({:.2f}, {:.2f})".format(
                self.waypoint_idx + 1, len(WAYPOINTS), name, x, y
            )
        )

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        _, _, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.goal_retry_count += 1
            if self.goal_retry_count <= self.MAX_GOAL_RETRIES:
                self.get_logger().warn(
                    "  目标被拒绝（第 {}/{} 次），{:.0f}s 后重试...".format(
                        self.goal_retry_count, self.MAX_GOAL_RETRIES,
                        self.GOAL_RETRY_DELAY
                    )
                )
                self.goal_retry_timer = self.create_timer(
                    self.GOAL_RETRY_DELAY, self._on_goal_retry)
                return
            self.get_logger().error("目标多次被拒绝，跳过此点。")
            self.goal_retry_count = 0
            self.waypoint_idx += 1
            self._navigate_next()
            return

        self.goal_retry_count = 0
        name = WAYPOINTS[self.waypoint_idx][0]
        self.get_logger().info("  → {} 目标已接受，行驶中...".format(name))
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_arrived)

    def _on_goal_retry(self):
        self.goal_retry_timer.cancel()
        self.goal_retry_timer = None
        self._navigate_next()

    def _on_arrived(self, future):
        name = WAYPOINTS[self.waypoint_idx][0]
        result = future.result()

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info("  ✓ 已到达 {}".format(name))
            # ── 到达后才做识别计数 ──
            self._do_detection(name)
        else:
            self.get_logger().warn(
                "  ✗ {} 导航失败 (status={}), 跳过".format(name, result.status)
            )

        self.waypoint_idx += 1
        self._navigate_next()

    # ── 视觉识别 ───────────────────────────────────────────────

    def _ensure_pipeline(self):
        """首次调用时加载 GroundingDINO + SAM2 模型（耗时，只做一次）。

        返回 True 表示可用；加载失败时记录错误并返回 False，保证巡逻继续，
        不会因为视觉模型问题把整个任务卡死在第一个观察点。
        """
        if self._pipeline is not None:
            return True
        self.get_logger().info(
            "  首次使用，正在加载 GroundingDINO + SAM2 模型（可能需要数十秒）..."
        )
        try:
            self._pipeline = vision_pipeline.VisionPipeline()
        except Exception as exc:
            self._pipeline = None
            self._pipeline_error = str(exc)
            self.get_logger().error("  ✗ 视觉模型加载失败: {}".format(exc))
            return False
        self.get_logger().info("  模型加载完成。")
        return True

    def _on_image_cb(self, msg):
        """相机图像回调：解码后缓存最新一帧 BGR。"""
        bgr = self._imgmsg_to_bgr(msg)
        if bgr is None:
            return
        with self._frame_lock:
            self._latest_frame = bgr

    def _on_depth_cb(self, msg):
        """深度图回调：缓存最新一帧 float 米制深度。"""
        depth = self._imgmsg_to_depth(msg)
        if depth is None:
            return
        with self._depth_lock:
            self._latest_depth = depth

    def _on_camera_info_cb(self, msg):
        with self._info_lock:
            self._camera_info = msg

    @staticmethod
    def _imgmsg_to_bgr(msg):
        """把 sensor_msgs/Image 手动转成 numpy BGR。

        刻意不用 cv_bridge：当前环境 numpy 2.x 与 ROS 的 cv_bridge（按
        numpy 1.x 编译）ABI 不兼容，导入即报 _ARRAY_API not found。
        """
        try:
            h, w = msg.height, msg.width
            data = np.frombuffer(msg.data, dtype=np.uint8)
        except Exception:
            return None

        enc = (msg.encoding or "").lower()
        if enc == "rgb8":
            return data.reshape(h, w, 3)[:, :, ::-1].copy()   # RGB -> BGR
        if enc == "bgr8":
            return data.reshape(h, w, 3).copy()
        if enc == "rgba8":
            return data.reshape(h, w, 4)[:, :, :3][:, :, ::-1].copy()
        if enc == "bgra8":
            return data.reshape(h, w, 4)[:, :, :3].copy()
        if enc in ("mono8", "8uc1"):
            return np.repeat(data.reshape(h, w)[..., None], 3, axis=2)

        # 兜底：按 step 推断通道数
        nch = msg.step // w if w else 3
        try:
            arr = data.reshape(h, w, nch)
        except ValueError:
            return None
        return arr[:, :, :3].copy()

    @staticmethod
    def _imgmsg_to_depth(msg):
        """把 sensor_msgs/Image（深度）转成 float 米制 numpy (H, W)。

        刻意不用 cv_bridge（numpy 2.x ABI 不兼容），手动按编码解析：
        rgbd_camera 的深度经 ros_gz_bridge 后通常是 32FC1（浮点米）。
        """
        try:
            h, w = msg.height, msg.width
            data = np.frombuffer(msg.data, dtype=np.uint8)
        except Exception:
            return None

        enc = (msg.encoding or "").lower()
        if enc == "32fc1":
            return data.view(np.float32).reshape(h, w).copy()
        if enc in ("16uc1", "mono16"):
            return data.view(np.uint16).reshape(h, w).astype(np.float32) / 1000.0
        return None

    def _depth_at_mask(self, depth, mask, cx, cy):
        """取掩码内深度中位数（忽略无效值），失败回退到中心 5×5 邻域。"""
        if depth is None:
            return None
        h, w = depth.shape
        if mask is not None and mask.shape[:2] == (h, w):
            vals = depth[mask]
            vals = vals[np.isfinite(vals) & (vals > 0)]
            if vals.size:
                return float(np.median(vals))
        # 回退：中心 5×5 邻域
        x0, y0 = max(0, int(round(cx)) - 2), max(0, int(round(cy)) - 2)
        x1, y1 = min(w, x0 + 5), min(h, y0 + 5)
        if x1 <= x0 or y1 <= y0:
            return None
        vals = depth[y0:y1, x0:x1]
        vals = vals[np.isfinite(vals) & (vals > 0)]
        if vals.size:
            return float(np.median(vals))
        return None

    def _resolve_camera_frame(self):
        """确定 depth/camera_info 光学帧在 TF 树里的真实名称。

        Gazebo 会给传感器帧加模型名前缀 turtlebot3/，而 TF 树里是未加前缀的
        帧名。这里依次尝试候选帧，返回第一个能从 map 查到的那个。
        """
        if self._camera_frame is not None:
            return self._camera_frame
        candidates = []
        with self._info_lock:
            info_frame = self._camera_info.header.frame_id if self._camera_info else ""
        if info_frame:
            candidates.append(info_frame)
        candidates += ["camera_rgb_optical_frame", "turtlebot3/camera_rgb_optical_frame"]
        for c in candidates:
            try:
                if self._tf_buffer.can_transform("map", c, Time()):
                    self._camera_frame = c
                    self.get_logger().info("  相机光学帧解析为: {}".format(c))
                    return c
            except Exception:
                continue
        self.get_logger().warn("  无法解析相机光学帧（TF 缺失），将跳过 3D 定位。")
        return None

    def _pixel_to_map(self, u, v, depth_m):
        """像素 (u,v) + 深度(米) → 相机光学帧 3D → /map 3D。失败返回 None。"""
        if depth_m is None or not np.isfinite(depth_m) or depth_m <= 0:
            return None
        with self._info_lock:
            info = self._camera_info
        if info is None:
            return None
        k = info.k  # [fx, 0, cx, 0, fy, cy, ...]
        fx, cx, fy, cy = k[0], k[2], k[4], k[5]
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m

        cam_frame = self._resolve_camera_frame()
        if cam_frame is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform("map", cam_frame, Time())
        except Exception as exc:
            self.get_logger().warn("  TF 查询失败 ({} → map): {}".format(cam_frame, exc))
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        px, py, pz = self._quat_rotate((q.x, q.y, q.z, q.w), (x, y, z))
        return (px + t.x, py + t.y, pz + t.z)

    @staticmethod
    def _quat_rotate(q, v):
        """用四元数 q=(x,y,z,w) 旋转向量 v=(x,y,z)，返回 (x',y',z')。"""
        qx, qy, qz, qw = q
        vx, vy, vz = v
        # v' = v + 2qw(qv×v) + 2(qv×(qv×v))
        c = (qy * vz - qz * vy, qz * vx - qx * vz, qx * vy - qy * vx)
        c2 = (qy * c[2] - qz * c[1], qz * c[0] - qx * c[2], qx * c[1] - qy * c[0])
        return (vx + 2 * qw * c[0] + 2 * c2[0],
                vy + 2 * qw * c[1] + 2 * c2[1],
                vz + 2 * qw * c[2] + 2 * c2[2])

    def _publish_markers(self):
        """发布累计识别物品的 Marker（/map 帧）。先 DELETEALL 再逐条 ADD。"""
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.ns = "detected_items"
        clear.action = Marker.DELETEALL
        self._marker_pub.publish(clear)

        for it in self._seen_objects:
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "detected_items"
            m.id = it["id"]
            m.action = Marker.ADD
            m.type = Marker.SPHERE
            m.pose.position.x = float(it["x"])
            m.pose.position.y = float(it["y"])
            m.pose.position.z = float(it["z"])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.10
            r, g, b = ITEM_COLORS.get(it["name"], (0.9, 0.9, 0.9))
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            self._marker_pub.publish(m)

    def _register_object(self, name, pos_3d):
        """按 /map 坐标去重并登记一个物品，返回 True 表示是新物品（需计数）。

        同一物理物体可能被相邻观察点各扫到一次（或把远处物体认错），这里用 /map
        坐标距离合并：距离 < DEDUP_DIST 视为同一物体。以**首次登记为准**（每个
        物品都在它自己桌子的观察点第一次被最近距离看到），后续重复检测直接丢弃、
        不覆盖类别。
        """
        x, y, z = pos_3d
        for obj in self._seen_objects:
            d = math.sqrt((x - obj["x"]) ** 2 + (y - obj["y"]) ** 2 + (z - obj["z"]) ** 2)
            if d < DEDUP_DIST:
                return False
        self._seen_objects.append({
            "id": self._next_marker_id, "name": name,
            "x": x, "y": y, "z": z,
        })
        self._next_marker_id += 1
        return True

    def _do_detection(self, table_name):
        """
        在当前观察点执行 GroundingDINO 检测 + SAM2 分割 + 目标中心点计算。

        流程：取最新相机帧 → 检测分割 → 计算每个实例掩码的像素中心
        → 结果存入 detection_results（含数量 + 每个目标中心点）。
        """
        self.get_logger().info("  🔍 {} 扫描中...".format(table_name))

        if not self._ensure_pipeline():
            self.detection_results[table_name] = {
                "count": 0, "objects": [],
                "error": getattr(self, "_pipeline_error", "model_load_failed"),
            }
            return

        with self._frame_lock:
            frame = self._latest_frame.copy() if self._latest_frame is not None else None

        if frame is None:
            self.get_logger().warn(
                "  ⚠ 尚未收到相机图像，跳过识别（请确认话题 {} 正在发布）。".format(
                    CAMERA_IMAGE_TOPIC)
            )
            self.detection_results[table_name] = {
                "count": 0, "objects": [], "reason": "no_image"
            }
            return

        try:
            detections = self._pipeline.detect(frame)
        except Exception as exc:
            self.get_logger().error("  ✗ 识别出错: {}".format(exc))
            self.detection_results[table_name] = {
                "count": 0, "objects": [], "error": str(exc)
            }
            return

        with self._depth_lock:
            depth = self._latest_depth.copy() if self._latest_depth is not None else None

        objects = []
        wp_counts = {name: 0 for name in vision_pipeline.ITEM_NAMES}
        counted_detections = []   # 真正参与计数的原始检测（画标注图只用这些）
        for d in detections:
            cx, cy = d["center_px"]
            name = vision_pipeline.classify_phrase(d["phrase"])

            # 像素中心 + 深度 → 相机光学帧 3D → /map 3D
            z_m = self._depth_at_mask(depth, d.get("mask"), cx, cy)
            pos_3d = self._pixel_to_map(cx, cy, z_m) if z_m is not None else None

            # 高度过滤：低于桌面的检测是桌腿/地面等结构，不是桌上物品，直接丢弃
            if pos_3d is not None and pos_3d[2] < MIN_OBJECT_Z:
                self.get_logger().info(
                    "      ⊘ {} 高度 {:.2f}m 低于桌面，判为误检跳过".format(
                        name or d["phrase"], pos_3d[2])
                )
                continue

            # 按 /map 坐标去重（相邻观察点可能扫到同一物体）
            is_new = True
            if name is not None and pos_3d is not None:
                is_new = self._register_object(name, pos_3d)

            objects.append({
                "phrase": d["phrase"],
                "name": name,
                "score": round(d["score"], 3),
                "center_px": [round(cx, 1), round(cy, 1)],
                "pos_map": None if pos_3d is None else
                           [round(pos_3d[0], 3), round(pos_3d[1], 3), round(pos_3d[2], 3)],
                "dup": not is_new,
            })

            # 计数（只有归到 ITEM_NAMES 之一且是新物体才算）
            if name is not None and is_new:
                self._item_counts[name] = self._item_counts.get(name, 0) + 1
                wp_counts[name] += 1
                counted_detections.append(d)   # 参与计数的原始检测，标注图只画这些

        self.detection_results[table_name] = {
            "count": len(objects),
            "objects": objects,
        }

        if self._seen_objects:
            self._publish_markers()

        self.get_logger().info(
            "  ✓ {} 识别完成：{} 个目标".format(table_name, len(objects))
        )
        for obj in objects:
            pos_txt = ("无" if obj["pos_map"] is None else
                       "({:.2f}, {:.2f}, {:.2f})".format(*obj["pos_map"]))
            dup_txt = " [重复,跳过]" if obj["dup"] else ""
            self.get_logger().info(
                "      - {} [{}] score={:.2f} 像素=({:.0f},{:.0f}) /map={}{}".format(
                    obj["phrase"], obj["name"] or "未归类", obj["score"],
                    obj["center_px"][0], obj["center_px"][1], pos_txt, dup_txt
                )
            )

        # 本次观察点三类物品计数
        parts = ", ".join("{}={}".format(n, wp_counts[n])
                          for n in vision_pipeline.ITEM_NAMES)
        self.get_logger().info("  📊 {} 本次计数: {}".format(table_name, parts))

        if depth is None or self._camera_info is None:
            self.get_logger().warn(
                "  ⚠ 深度图或相机内参不可用，已跳过 3D 定位（仅计数）。"
            )

        if SAVE_DEBUG_IMAGE and counted_detections:
            self._save_debug_image(table_name, frame, counted_detections)

    def _save_debug_image(self, table_name, frame, detections):
        os.makedirs(DEBUG_IMAGE_DIR, exist_ok=True)
        path = os.path.join(DEBUG_IMAGE_DIR, "{}.jpg".format(table_name))
        self._pipeline.save_annotated(path, frame, detections)
        self.get_logger().info("  📷 结果图已保存: {}".format(path))

    # ── 完成 ───────────────────────────────────────────────────

    def _all_done(self):
        self.get_logger().info("=" * 45)
        self.get_logger().info("所有观察点扫描完成")
        self.get_logger().info("检测结果: {}".format(self.detection_results))
        self.get_logger().info("=" * 45)

        # 终端打印各待计数物品的英文名称与数量
        print("\n===== 计数结果（待计数物品）=====", flush=True)
        for name in vision_pipeline.ITEM_NAMES:
            print("  {}: {}".format(name, self._item_counts.get(name, 0)), flush=True)
        print("====================================\n", flush=True)
        self.get_logger().info("累计计数: {}".format(self._item_counts))


def main():
    print("=== patrol_task starting ===", flush=True)
    rclpy.init()
    node = CompetitionTask()
    print("=== node created, waiting for AMCL... ===", flush=True)

    # 等待 AMCL 初始定位收敛
    time.sleep(5.0)

    # 设置初始位姿 + 等待 Nav2 + 开始巡逻
    node.start()

    # start() 内部的导航结果通过 action callback 异步处理
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

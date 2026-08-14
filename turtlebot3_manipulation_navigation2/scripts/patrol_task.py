#!/usr/bin/env python3
"""
任务节点 — 初始定位 → 巡逻识别计数 → 汇总输出

执行流程:
  1. 发布初始位姿到 /initialpose
  2. 等待 Nav2 就绪
  3. 依次导航到 4 个观察点（到达 → 识别计数 → 下一个）
  4. 汇总输出 + RViz Marker

用法:
  ros2 run turtlebot3_manipulation_navigation2 patrol_task.py
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


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
    ("table_3", -4.7, -3.1, 0.0),           # 面朝 +X（living_room_table_3）
    ("table_1", -3.3, -1.2, - math.pi / 2), # 面朝 -Y（living_room_table_1）
    ("table_0", -2.1, -1.2, math.pi / 2),   # 面朝 +Y（living_room_table_0）
    ("table_2", -0.6, -2.6, 0.0),           # 面朝 +X（living_room_table_2）
]


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

    def _do_detection(self, table_name):
        """
        在当前观察点执行物体检测 + 计数。
        后续接入 GroundingDINO + 分类模型。
        """
        self.get_logger().info("  🔍 扫描中...")
        # TODO: 订阅 /pi_camera/image → GroundingDINO 检测 → 分类确认 → 去重计数
        # TODO: 深度图 + TF → 计算 /map 坐标 → 发布 RViz Marker
        time.sleep(2.0)
        self.get_logger().info("  ✓ {} 扫描完成".format(table_name))

    # ── 完成 ───────────────────────────────────────────────────

    def _all_done(self):
        self.get_logger().info("=" * 45)
        self.get_logger().info("所有观察点扫描完成")
        self.get_logger().info("检测结果: {}".format(self.detection_results))
        self.get_logger().info("=" * 45)


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

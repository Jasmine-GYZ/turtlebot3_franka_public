#!/usr/bin/env python3
"""将 Nav2 输出的 /cmd_vel 桥接到 diff_drive_controller 的输入话题。

diff_drive_controller（use_stamped_vel: false）订阅命名空间下的
/diff_drive_controller/cmd_vel_unstamped，而 Nav2 的 velocity_smoother
发布到 /cmd_vel。二者 QoS 均为默认（reliable/volatile），直接转发即可。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")
        self.pub = self.create_publisher(
            Twist, "/diff_drive_controller/cmd_vel_unstamped", 10)
        self.sub = self.create_subscription(
            Twist, "/cmd_vel", self._cb, 10)

    def _cb(self, msg: Twist):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

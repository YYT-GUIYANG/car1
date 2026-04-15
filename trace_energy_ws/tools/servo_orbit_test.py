#!/usr/bin/env python3
"""
固定速度环绕测试（不依赖视觉）。

用途：
- 仅验证串口链路、舵机ID映射、机械方向和速度边界。
- 通过 /servo_control 连续发布俯仰+偏航目标角度。

使用前提：
1) 已 source ROS2 与工作区 install/setup.bash
2) servo_uart_node 正在运行
3) 当前会话具备串口权限（例如 newgrp dialout）
"""

import argparse
import math
import signal
import sys

import rclpy
from rclpy.node import Node
from servo_message.msg import ServoMessage


def clamp_angle(v: float) -> int:
    return int(max(0, min(180, round(v))))


class ServoOrbitPublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("servo_orbit_test_pub")
        self.args = args
        self.pub = self.create_publisher(ServoMessage, "/servo_control", 10)

        self.phase = 0.0
        self.dt = 1.0 / float(max(1.0, args.rate))
        self.omega = 2.0 * math.pi * float(args.hz)
        self.elapsed = 0.0
        self.stopped = False

        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info(
            "orbit test start: pitch_id=%d yaw_id=%d hz=%.3f rate=%.1f amp=(%.1f,%.1f) center=(%.1f,%.1f) duration=%.1fs"
            % (
                args.id_pitch,
                args.id_yaw,
                args.hz,
                args.rate,
                args.amp_pitch,
                args.amp_yaw,
                args.center_pitch,
                args.center_yaw,
                args.duration,
            )
        )

    def _publish_angle(self, sid: int, angle: int) -> None:
        msg = ServoMessage()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.servo_id = int(sid)
        msg.servo_angle = int(angle)
        self.pub.publish(msg)

    def on_timer(self) -> None:
        if self.stopped:
            return

        yaw = self.args.center_yaw + self.args.amp_yaw * math.sin(self.phase)
        # 俯仰使用相位差，避免双轴同相导致轨迹退化成直线
        pitch = self.args.center_pitch + self.args.amp_pitch * math.cos(self.phase + self.args.phase_offset)

        yaw_i = clamp_angle(yaw)
        pitch_i = clamp_angle(pitch)

        self._publish_angle(self.args.id_yaw, yaw_i)
        self._publish_angle(self.args.id_pitch, pitch_i)

        self.phase += self.omega * self.dt
        self.elapsed += self.dt
        if self.elapsed >= self.args.duration:
            self.stop_and_center()

    def stop_and_center(self) -> None:
        if self.stopped:
            return
        self.stopped = True
        self._publish_angle(self.args.id_yaw, clamp_angle(self.args.center_yaw))
        self._publish_angle(self.args.id_pitch, clamp_angle(self.args.center_pitch))
        self.get_logger().info("orbit test done, servos centered.")
        self.destroy_timer(self.timer)
        rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="固定速度双轴环绕测试发布器")
    p.add_argument("--id-yaw", type=int, default=11, help="偏航舵机ID（默认11）")
    p.add_argument("--id-pitch", type=int, default=8, help="俯仰舵机ID（默认8）")
    p.add_argument("--center-yaw", type=float, default=90.0, help="偏航中心角")
    p.add_argument("--center-pitch", type=float, default=90.0, help="俯仰中心角")
    p.add_argument("--amp-yaw", type=float, default=12.0, help="偏航振幅（度）")
    p.add_argument("--amp-pitch", type=float, default=8.0, help="俯仰振幅（度）")
    p.add_argument("--hz", type=float, default=0.18, help="环绕频率（Hz）")
    p.add_argument("--rate", type=float, default=25.0, help="发布频率（Hz）")
    p.add_argument("--duration", type=float, default=20.0, help="测试时长（秒）")
    p.add_argument("--phase-offset", type=float, default=1.5708, help="俯仰相位偏移（弧度）")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = ServoOrbitPublisher(args)

    def _handle_sig(_sig, _frm):
        node.stop_and_center()

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_and_center()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())


#!/usr/bin/env python3
"""
视觉赛分阶段舵机运动测试（不依赖视觉）：
1) 仅左右（yaw）
2) 仅俯仰（pitch）
3) 双轴同时环绕
4) 回中并结束
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


class ServoRaceSequencePublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("servo_race_sequence_test_pub")
        self.args = args
        self.pub = self.create_publisher(ServoMessage, "/servo_control", 20)

        self.dt = 1.0 / float(max(1.0, args.rate))
        self.omega = 2.0 * math.pi * float(args.hz)
        self.phase = 0.0
        self.elapsed = 0.0
        self.stopped = False
        self.last_stage = ""

        self.timer = self.create_timer(self.dt, self.on_timer)
        self.get_logger().info(
            (
                "sequence test start: yaw_id=%d pitch_id=%d "
                "center=(%.1f,%.1f) amp=(%.1f,%.1f) hz=%.3f rate=%.1f "
                "durations(yaw=%.1f,pitch=%.1f,both=%.1f,hold=%.1f)"
            )
            % (
                args.id_yaw,
                args.id_pitch,
                args.center_yaw,
                args.center_pitch,
                args.amp_yaw,
                args.amp_pitch,
                args.hz,
                args.rate,
                args.duration_yaw,
                args.duration_pitch,
                args.duration_both,
                args.duration_hold,
            )
        )

    def _publish_angle(self, sid: int, angle: int) -> None:
        msg = ServoMessage()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.servo_id = int(sid)
        msg.servo_angle = int(angle)
        self.pub.publish(msg)

    def _get_stage(self) -> str:
        t = self.elapsed
        t1 = self.args.duration_yaw
        t2 = t1 + self.args.duration_pitch
        t3 = t2 + self.args.duration_both
        t4 = t3 + self.args.duration_hold
        if t < t1:
            return "yaw_only"
        if t < t2:
            return "pitch_only"
        if t < t3:
            return "both_orbit"
        if t < t4:
            return "hold_center"
        return "done"

    def on_timer(self) -> None:
        if self.stopped:
            return

        stage = self._get_stage()
        if stage != self.last_stage:
            self.last_stage = stage
            self.get_logger().info(f"stage -> {stage}")

        if stage == "done":
            self.stop_and_center()
            return

        yaw = self.args.center_yaw
        pitch = self.args.center_pitch

        if stage == "yaw_only":
            yaw = self.args.center_yaw + self.args.amp_yaw * math.sin(self.phase)
        elif stage == "pitch_only":
            pitch = self.args.center_pitch + self.args.amp_pitch * math.sin(self.phase)
        elif stage == "both_orbit":
            yaw = self.args.center_yaw + self.args.amp_yaw * math.sin(self.phase)
            pitch = self.args.center_pitch + self.args.amp_pitch * math.cos(
                self.phase + self.args.phase_offset
            )

        self._publish_angle(self.args.id_yaw, clamp_angle(yaw))
        self._publish_angle(self.args.id_pitch, clamp_angle(pitch))

        self.phase += self.omega * self.dt
        self.elapsed += self.dt

    def stop_and_center(self) -> None:
        if self.stopped:
            return
        self.stopped = True
        self._publish_angle(self.args.id_yaw, clamp_angle(self.args.center_yaw))
        self._publish_angle(self.args.id_pitch, clamp_angle(self.args.center_pitch))
        self.get_logger().info("sequence test done, servos centered.")
        self.destroy_timer(self.timer)
        rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="视觉赛分阶段舵机长时测试")
    p.add_argument("--id-yaw", type=int, default=11, help="偏航舵机ID")
    p.add_argument("--id-pitch", type=int, default=8, help="俯仰舵机ID")
    p.add_argument("--center-yaw", type=float, default=90.0, help="偏航中心角")
    p.add_argument("--center-pitch", type=float, default=90.0, help="俯仰中心角")
    p.add_argument("--amp-yaw", type=float, default=20.0, help="偏航振幅")
    p.add_argument("--amp-pitch", type=float, default=16.0, help="俯仰振幅")
    p.add_argument("--hz", type=float, default=0.18, help="运动频率（Hz）")
    p.add_argument("--rate", type=float, default=25.0, help="发布频率（Hz）")
    p.add_argument("--duration-yaw", type=float, default=18.0, help="仅左右时长（s）")
    p.add_argument("--duration-pitch", type=float, default=18.0, help="仅俯仰时长（s）")
    p.add_argument("--duration-both", type=float, default=24.0, help="双轴时长（s）")
    p.add_argument("--duration-hold", type=float, default=3.0, help="末尾回中保持（s）")
    p.add_argument("--phase-offset", type=float, default=1.5708, help="双轴相位偏移")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = ServoRaceSequencePublisher(args)

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


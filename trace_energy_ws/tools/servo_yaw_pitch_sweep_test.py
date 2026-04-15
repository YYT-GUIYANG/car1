#!/usr/bin/env python3
"""
诊断：偏航 / 俯仰各自缓慢扫过约 90°，确认串口与舵机号是否正常、是否顶行程。

使用前请先停掉 trace_calculator（或整个 launch），否则会与视觉节点抢发 /servo_control。

示例（若在 conda base 下请先 conda deactivate，否则会因 Python 3.12 无法加载 rclpy）：
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  bash tools/run_with_ros2_python.sh tools/servo_yaw_pitch_sweep_test.py

或显式使用系统 3.10：
  /usr/bin/python3.10 tools/servo_yaw_pitch_sweep_test.py

默认与 trace_calculator 一致：pitch_id=8, yaw_id=11；小幅慢扫（约 ±22°）便于观察 11 号行程。
全行程测试请加：--yaw-delta 90 --pitch-delta 90
"""
from __future__ import annotations

import argparse
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from servo_message.msg import ServoMessage
except ModuleNotFoundError as exc:
    if "_rclpy" in str(exc) or "rclpy" in str(exc):
        sys.stderr.write(
            "\n[rclpy] 导入失败（常见于 conda 的 Python 3.12 与 ROS Humble 的 3.10 扩展不匹配）。\n"
            "  1) conda deactivate\n"
            "  2) source /opt/ros/humble/setup.bash && source install/setup.bash\n"
            "  3) bash tools/run_with_ros2_python.sh tools/servo_yaw_pitch_sweep_test.py ...\n"
            "  或: /usr/bin/python3.10 tools/servo_yaw_pitch_sweep_test.py ...\n\n"
        )
    raise SystemExit(2) from exc


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def main() -> int:
    p = argparse.ArgumentParser(description="缓慢扫 yaw 再扫 pitch，用于舵机/行程诊断")
    p.add_argument("--yaw-id", type=int, default=11, help="偏航舵机 ID（与 trace_calculator servo_id_yaw 一致）")
    p.add_argument("--pitch-id", type=int, default=8, help="俯仰舵机 ID")
    p.add_argument("--yaw-from", type=float, default=90.0, help="偏航起始角度 0~180")
    p.add_argument("--pitch-from", type=float, default=55.0, help="俯仰起始角度 0~180")
    p.add_argument("--yaw-delta", type=float, default=22.0, help="第一阶段偏航总变化量（正为增大）")
    p.add_argument("--pitch-delta", type=float, default=22.0, help="第二阶段俯仰总变化量")
    p.add_argument("--step-deg", type=float, default=0.8, help="每步角度（越小越慢）")
    p.add_argument("--step-period", type=float, default=0.045, help="每步间隔秒")
    p.add_argument("--settle", type=float, default=0.35, help="两阶段之间停顿秒数")
    p.add_argument("--topic", type=str, default="servo_control", help="发布话题名")
    args = p.parse_args()

    print(
        "\n*** 请先停止 trace_calculator / 完整 launch，再运行本脚本，否则会抢发舵机话题 ***\n",
        file=sys.stderr,
    )

    rclpy.init()
    node = Node("servo_yaw_pitch_sweep_test")
    pub = node.create_publisher(ServoMessage, args.topic, 10)
    time.sleep(0.3)  # 等 DDS 建连

    def send_one(sid: int, ang: int) -> None:
        m = ServoMessage()
        m.servo_id = int(sid)
        m.servo_angle = int(clamp(float(ang), 0.0, 180.0))
        pub.publish(m)

    def ramp_one_axis(
        axis_id: int,
        other_id: int,
        other_hold: int,
        a0: float,
        a1: float,
        label: str,
    ) -> None:
        a0c = clamp(a0, 0.0, 180.0)
        a1c = clamp(a1, 0.0, 180.0)
        total = abs(a1c - a0c)
        if total < 1e-6:
            print(f"[{label}] 起止相同，跳过", file=sys.stderr)
            return
        step = args.step_deg if a1c >= a0c else -args.step_deg
        n = max(1, int(round(total / abs(args.step_deg))))
        print(f"[{label}] {a0c:.0f}° → {a1c:.0f}° 共约 {n} 步", file=sys.stderr)
        for i in range(n + 1):
            t = i / float(n)
            a = a0c + (a1c - a0c) * t
            ang = int(round(a))
            send_one(axis_id, ang)
            send_one(other_id, other_hold)
            time.sleep(args.step_period)
        # 最后再发一次终点，确保到位
        send_one(axis_id, int(round(a1c)))
        send_one(other_id, other_hold)
        time.sleep(0.05)

    yaw0 = clamp(args.yaw_from, 0.0, 180.0)
    pitch0 = clamp(args.pitch_from, 0.0, 180.0)
    yaw1 = clamp(yaw0 + args.yaw_delta, 0.0, 180.0)
    pitch1 = clamp(pitch0 + args.pitch_delta, 0.0, 180.0)

    print(
        f"阶段1：仅偏航 id={args.yaw_id}  {yaw0:.0f}°→{yaw1:.0f}°，俯仰 id={args.pitch_id} 保持 {pitch0:.0f}°",
        file=sys.stderr,
    )
    ramp_one_axis(args.yaw_id, args.pitch_id, int(round(pitch0)), yaw0, yaw1, "YAW")

    time.sleep(args.settle)

    yaw_hold = int(round(yaw1))
    print(
        f"阶段2：仅俯仰 id={args.pitch_id}  {pitch0:.0f}°→{pitch1:.0f}°，偏航 id={args.yaw_id} 保持 {yaw_hold}°",
        file=sys.stderr,
    )
    ramp_one_axis(args.pitch_id, args.yaw_id, yaw_hold, pitch0, pitch1, "PITCH")

    print("完成。若某一轴几乎不动：检查该轴舵机号、接线、以及是否已在机械极限。", file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

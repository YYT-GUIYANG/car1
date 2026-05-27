
#!/usr/bin/env python3
"""
从话题 /trace_debug_image 订阅最多 wait_sec 秒，保存两张处理后的图（BGR8）。
用法（须先 conda deactivate 再 source ROS）:
  python3 tools/capture_trace_debug_snapshots.py
  python3 tools/capture_trace_debug_snapshots.py --wait 30 --out /tmp/trace
无 ROS 时加 --demo 仅生成本地两张合成示意图（风格与调试图一致）。
"""
from __future__ import annotations

import argparse
import os
import sys
import time


def save_demo_frames(out_prefix: str) -> None:
    import cv2
    import numpy as np

    w, h = 960, 720
    for idx in (1, 2):
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        frame[:] = (40, 42, 48)
        rw, rh = int(w * 0.70), int(h * 0.70)
        x0, y0 = (w - rw) // 2, (h - rh) // 2
        cv2.rectangle(frame, (x0, y0), (x0 + rw, y0 + rh), (60, 80, 100), 2)
        cx, cy = w // 2 + (idx - 1) * 8, h // 2 - 5
        r = 62
        cv2.circle(frame, (cx, cy), r, (180, 90, 140), -1)
        cv2.circle(frame, (cx, cy), r, (0, 220, 60), 2)
        pts = np.array(
            [[cx + 140, cy - 20], [cx + 220, cy - 10], [cx + 210, cy + 55], [cx + 125, cy + 40]],
            np.int32,
        )
        cv2.polylines(frame, [pts], True, (0, 200, 255), 2)
        cv2.putText(
            frame,
            f"DEMO processed-like frame {idx}/2  roi=0.70  Hough+minAreaRect",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (200, 230, 255),
            2,
        )
        cv2.putText(
            frame,
            "subscribe: /trace_debug_image (run trace + camera for real)",
            (12, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (180, 200, 220),
            2,
        )
        path = f"{out_prefix}_{idx:02d}.png"
        cv2.imwrite(path, frame)
        print(path, flush=True)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--wait", type=float, default=30.0, help="最长等待秒数")
    p.add_argument("--topic", type=str, default="/trace_debug_image")
    p.add_argument("--out", type=str, default="trace_debug_capture", help="输出前缀（不含扩展名）")
    p.add_argument("--demo", action="store_true", help="不连 ROS，写两张本地合成示意图")
    args = p.parse_args()
    out_abs = os.path.abspath(args.out)

    if args.demo:
        save_demo_frames(out_abs)
        return 0

    try:
        import rclpy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
    except ModuleNotFoundError as e:
        print("缺少 rclpy/cv_bridge，请 conda deactivate 后 source ROS，或使用 --demo", file=sys.stderr)
        print(e, file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node("trace_debug_capture")
    bridge = CvBridge()
    saved: list[str] = []
    t_end = time.time() + args.wait

    def cb(msg: Image) -> None:
        if len(saved) >= 2:
            return
        try:
            im = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as ex:  # noqa: BLE001
            node.get_logger().warn(f"cv_bridge: {ex}")
            return
        path = f"{out_abs}_{len(saved) + 1:02d}.png"
        import cv2

        cv2.imwrite(path, im)
        saved.append(path)
        node.get_logger().info(f"saved {path}")

    node.create_subscription(Image, args.topic, cb, 10)
    node.get_logger().info(f"waiting up to {args.wait}s on {args.topic} ...")
    while rclpy.ok() and time.time() < t_end and len(saved) < 2:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    if len(saved) < 2:
        print(f"仅保存 {len(saved)} 张，请确认 trace 已发布 {args.topic}", file=sys.stderr)
        return 1
    for s in saved:
        print(s, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

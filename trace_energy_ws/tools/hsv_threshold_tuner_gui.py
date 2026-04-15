#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BGR 图像 HSV 阈值滑条标定（OpenCV）。

用法示例：
  # 本地图
  python3 hsv_threshold_tuner_gui.py --image /path/to/snap.png

  # ROS2 订阅（需系统 ROS Python，勿用 conda 覆盖的 python）
  source /opt/ros/humble/setup.bash
  python3 hsv_threshold_tuner_gui.py --topic /processed_image

窗口内 H/S/V 低高共 6 条 trackbar；左侧为原图，右侧为掩膜+轮廓。
终端会周期性打印当前 HSV 范围，便于抄到别的脚本或笔记。
"""

from __future__ import annotations

import argparse
import sys
import time


def _print_range(h0, h1, s0, s1, v0, v1):
    # OpenCV H 0..179
    print(
        f"[HSV] lower=({h0},{s0},{v0}) upper=({h1},{s1},{v1})  "
        f"# OpenCV H in 0..179",
        flush=True,
    )


def _run_file(image_path: str):
    import cv2
    import numpy as np

    bgr = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if bgr is None or bgr.size == 0:
        print(f"无法读取图像: {image_path}", file=sys.stderr)
        sys.exit(1)

    win = "hsv_tuner"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    state = {"h0": 0, "h1": 179, "s0": 0, "s1": 255, "v0": 0, "v1": 255}

    def _on(_v, key):
        pass

    for k, lo, hi in (
        ("H_low", 0, 179),
        ("H_high", 0, 179),
        ("S_low", 0, 255),
        ("S_high", 0, 255),
        ("V_low", 0, 255),
        ("V_high", 0, 255),
    ):
        cv2.createTrackbar(k, win, lo, hi, _on)
    cv2.setTrackbarPos("H_low", win, state["h0"])
    cv2.setTrackbarPos("H_high", win, state["h1"])
    cv2.setTrackbarPos("S_low", win, state["s0"])
    cv2.setTrackbarPos("S_high", win, state["s1"])
    cv2.setTrackbarPos("V_low", win, state["v0"])
    cv2.setTrackbarPos("V_high", win, state["v1"])

    last_print = 0.0
    while True:
        h0 = cv2.getTrackbarPos("H_low", win)
        h1 = cv2.getTrackbarPos("H_high", win)
        s0 = cv2.getTrackbarPos("S_low", win)
        s1 = cv2.getTrackbarPos("S_high", win)
        v0 = cv2.getTrackbarPos("V_low", win)
        v1 = cv2.getTrackbarPos("V_high", win)
        if h0 > h1:
            h0, h1 = h1, h0
        if s0 > s1:
            s0, s1 = s1, s0
        if v0 > v1:
            v0, v1 = v1, v0

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lower = np.array([h0, s0, v0], dtype=np.uint8)
        upper = np.array([h1, s1, v1], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, cnts, -1, (0, 255, 0), 1)
        show = np.hstack([bgr, vis])
        cv2.imshow(win, show)

        now = time.time()
        if now - last_print > 1.2:
            _print_range(h0, h1, s0, s1, v0, v1)
            last_print = now

        k = cv2.waitKey(30) & 0xFF
        if k in (27, ord("q")):
            break
    cv2.destroyAllWindows()


def _run_ros(topic: str):
    import cv2
    import numpy as np

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
    except ImportError as e:
        print(
            "ROS2 依赖缺失（rclpy/cv_bridge）。请 source ROS setup 且勿用 conda 的 python。\n"
            f"详情: {e}",
            file=sys.stderr,
        )
        sys.exit(2)

    class Sub(Node):
        def __init__(self):
            super().__init__("hsv_threshold_tuner_gui")
            self._bridge = CvBridge()
            self._last = None
            self.create_subscription(Image, topic, self._cb, 10)

        def _cb(self, msg: Image):
            try:
                self._last = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as ex:  # noqa: BLE001
                self.get_logger().warn(f"cv_bridge 转换失败: {ex}")

        @property
        def frame(self):
            return None if self._last is None else self._last

    rclpy.init()
    node = Sub()
    win = "hsv_tuner"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    def _on(_v):
        pass

    for k, lo, hi in (
        ("H_low", 0, 179),
        ("H_high", 0, 179),
        ("S_low", 0, 255),
        ("S_high", 0, 255),
        ("V_low", 0, 255),
        ("V_high", 0, 255),
    ):
        cv2.createTrackbar(k, win, lo, hi, _on)
    cv2.setTrackbarPos("H_low", win, 0)
    cv2.setTrackbarPos("H_high", win, 179)
    cv2.setTrackbarPos("S_low", win, 0)
    cv2.setTrackbarPos("S_high", win, 255)
    cv2.setTrackbarPos("V_low", win, 0)
    cv2.setTrackbarPos("V_high", win, 255)

    last_print = 0.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.02)
        bgr = node.frame
        if bgr is None:
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                blank,
                f"waiting {topic}...",
                (24, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (200, 200, 200),
                2,
            )
            cv2.imshow(win, blank)
            k = cv2.waitKey(30) & 0xFF
            if k in (27, ord("q")):
                break
            continue

        h0 = cv2.getTrackbarPos("H_low", win)
        h1 = cv2.getTrackbarPos("H_high", win)
        s0 = cv2.getTrackbarPos("S_low", win)
        s1 = cv2.getTrackbarPos("S_high", win)
        v0 = cv2.getTrackbarPos("V_low", win)
        v1 = cv2.getTrackbarPos("V_high", win)
        if h0 > h1:
            h0, h1 = h1, h0
        if s0 > s1:
            s0, s1 = s1, s0
        if v0 > v1:
            v0, v1 = v1, v0

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        lower = np.array([h0, s0, v0], dtype=np.uint8)
        upper = np.array([h1, s1, v1], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, cnts, -1, (0, 255, 0), 1)
        # 缩放到便于显示
        max_w = 1280
        if bgr.shape[1] * 2 > max_w:
            sc = max_w / float(bgr.shape[1] * 2)
            bgr_s = cv2.resize(bgr, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
            vis_s = cv2.resize(vis, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
            show = np.hstack([bgr_s, vis_s])
        else:
            show = np.hstack([bgr, vis])
        cv2.imshow(win, show)

        now = time.time()
        if now - last_print > 1.2:
            _print_range(h0, h1, s0, s1, v0, v1)
            last_print = now

        k = cv2.waitKey(30) & 0xFF
        if k in (27, ord("q")):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description="HSV 阈值滑条标定（BGR）")
    ap.add_argument("--image", default="", help="本地 BGR 图像路径")
    ap.add_argument("--topic", default="", help="ROS2 图像话题（如 /processed_image）")
    args = ap.parse_args()
    if bool(args.image) == bool(args.topic):
        ap.error("请只指定其一：--image 或 --topic")
    if args.image:
        _run_file(args.image)
    else:
        _run_ros(args.topic)


if __name__ == "__main__":
    main()

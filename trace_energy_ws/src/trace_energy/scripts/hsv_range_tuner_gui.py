#!/usr/bin/env python3
"""
HSV 阈值滑动条调参工具：实时看掩膜与 inRange 结果，便于高光/阴影/边缘综合设限。
用法:
  conda deactivate
  source /opt/ros/humble/setup.bash
  ros2 run trace_energy hsv_range_tuner_gui -- --camera 0
  ros2 run trace_energy hsv_range_tuner_gui -- --topic /processed_image
按键: q 退出；鼠标左键在画面上点击可将当前像素 HSV 填到「以采样为中心」的窄窗（再手动拖条扩宽）。
"""
from __future__ import annotations

import argparse
import sys


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--camera", type=int, default=-1, help=">=0 时用本机摄像头索引；-1 表示不用摄像头")
    p.add_argument("--topic", type=str, default="", help="非空则订阅 ROS 图像话题（需先 conda deactivate + source ROS）")
    p.add_argument("--width", type=int, default=960)
    args = p.parse_args()

    import cv2
    import numpy as np

    win = "HSV tuner (mask | original)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    h0, h1, s0, s1, v0, v1 = 0, 179, 0, 255, 0, 255
    cv2.createTrackbar("Hmin", win, 0, 179, lambda x: None)
    cv2.createTrackbar("Hmax", win, 179, 179, lambda x: None)
    cv2.createTrackbar("Smin", win, 0, 255, lambda x: None)
    cv2.createTrackbar("Smax", win, 255, 255, lambda x: None)
    cv2.createTrackbar("Vmin", win, 0, 255, lambda x: None)
    cv2.createTrackbar("Vmax", win, 255, 255, lambda x: None)

    cap = None
    sub = None
    node = None
    last_bgr: np.ndarray | None = None

    if args.topic:
        try:
            import rclpy
            from cv_bridge import CvBridge
            from sensor_msgs.msg import Image
        except ModuleNotFoundError as e:
            print("需要 ROS Python：conda deactivate 后 source /opt/ros/humble/setup.bash", file=sys.stderr)
            print(e, file=sys.stderr)
            return 2

        rclpy.init()
        node = rclpy.create_node("hsv_tuner_sub")
        bridge = CvBridge()

        def cb(msg: Image) -> None:
            nonlocal last_bgr
            try:
                last_bgr = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                pass

        sub = node.create_subscription(Image, args.topic, cb, 10)
    elif args.camera >= 0:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(args.camera)
        if not cap.isOpened():
            print(f"无法打开摄像头 {args.camera}", file=sys.stderr)
            return 1
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)

    def read_frame() -> np.ndarray | None:
        if cap is not None:
            ok, fr = cap.read()
            return fr if ok else None
        if node is not None:
            rclpy.spin_once(node, timeout_sec=0.05)
            return last_bgr
        print("请指定 --camera N 或 --topic /...", file=sys.stderr)
        return None

    def on_mouse(event, x, y, flags, param) -> None:  # noqa: ARG001
        nonlocal h0, h1, s0, s1, v0, v1
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        fr = param.get("frame")
        if fr is None or y < 0 or x < 0 or y >= fr.shape[0] or x >= fr.shape[1]:
            return
        hsv = cv2.cvtColor(fr[y : y + 1, x : x + 1], cv2.COLOR_BGR2HSV)
        hh, ss, vv = int(hsv[0, 0, 0]), int(hsv[0, 0, 1]), int(hsv[0, 0, 2])
        dh, ds, dv = 10, 40, 40
        h0, h1 = max(0, hh - dh), min(179, hh + dh)
        s0, s1 = max(0, ss - ds), min(255, ss + ds)
        v0, v1 = max(0, vv - dv), min(255, vv + dv)
        cv2.setTrackbarPos("Hmin", win, h0)
        cv2.setTrackbarPos("Hmax", win, h1)
        cv2.setTrackbarPos("Smin", win, s0)
        cv2.setTrackbarPos("Smax", win, s1)
        cv2.setTrackbarPos("Vmin", win, v0)
        cv2.setTrackbarPos("Vmax", win, v1)

    cv2.setMouseCallback(win, on_mouse, {"frame": None})

    while True:
        frame = read_frame()
        if frame is None:
            if cap is None and node is not None:
                continue
            break
        if frame.shape[1] > args.width:
            sc = args.width / frame.shape[1]
            frame = cv2.resize(frame, (int(frame.shape[1] * sc), int(frame.shape[0] * sc)))
        h0 = cv2.getTrackbarPos("Hmin", win)
        h1 = cv2.getTrackbarPos("Hmax", win)
        s0 = cv2.getTrackbarPos("Smin", win)
        s1 = cv2.getTrackbarPos("Smax", win)
        v0 = cv2.getTrackbarPos("Vmin", win)
        v1 = cv2.getTrackbarPos("Vmax", win)
        if h1 < h0:
            h0, h1 = h1, h0
        if s1 < s0:
            s0, s1 = s1, s0
        if v1 < v0:
            v0, v1 = v1, v0
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array([h0, s0, v0], dtype=np.uint8)
        upper = np.array([h1, s1, v1], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        blend = cv2.addWeighted(frame, 0.45, vis, 0.55, 0)
        cv2.putText(
            blend,
            f"H[{h0},{h1}] S[{s0},{s1}] V[{v0},{v1}]  click=sample",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
        )
        cv2.setMouseCallback(win, on_mouse, {"frame": frame})
        cv2.imshow(win, blend)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break

    if cap:
        cap.release()
    if node:
        node.destroy_node()
        rclpy.shutdown()
    cv2.destroyAllWindows()
    print(f"最后 HSV 范围: H[{h0},{h1}] S[{s0},{s1}] V[{v0},{v1}]", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

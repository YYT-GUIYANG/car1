#!/usr/bin/env python3
"""
无相机烟测：向 trace_calculator 发布纯色帧，检查丢目标后 tracking_debug 中俯仰角是否出现搜索摆动。

依赖：rclpy、sensor_msgs、std_msgs（ROS2 Humble 已带）。
用法（在 trace_energy_ws 下，且已 source install/setup.bash）：

  export LD_LIBRARY_PATH=<servo_message 的 lib 目录>:$LD_LIBRARY_PATH   # 若 ldd 缺 libservo_message
  # 终端1：ros2 run trace_energy trace_calculator --ros-args -p control_enabled:=false -p startup_hold_frames:=0 ...
  # 终端2：
  python3 tools/search_mode_synthetic_test.py

或一键：tools/run_search_smoke_test.sh
"""
from __future__ import annotations

import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray


def _make_gray_bgr(h: int = 480, w: int = 640, b: int = 45, g: int = 45, r: int = 45) -> bytes:
    row = bytes([b, g, r] * w)
    return row * h


class SynthTestNode(Node):
    def __init__(self, image_topic: str):
        super().__init__("search_mode_synthetic_test")
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(Image, image_topic, qos)
        self._dbg: list[tuple[float, float, float]] = []  # (t, pitch, yaw)
        self.create_subscription(
            Float64MultiArray,
            "/tracking_debug",
            self._on_dbg,
            20,
        )
        self._img = Image()
        self._img.height = 480
        self._img.width = 640
        self._img.encoding = "bgr8"
        self._img.is_bigendian = 0
        self._img.step = 640 * 3
        self._img.data = _make_gray_bgr()
        self._period = 1.0 / 30.0
        self._timer = self.create_timer(self._period, self._tick)

    def _on_dbg(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 8:
            return
        pitch = float(msg.data[6])
        yaw = float(msg.data[7])
        self._dbg.append((time.monotonic(), pitch, yaw))

    def _tick(self) -> None:
        self._img.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._img)


def discover_image_topic(timeout_sec: float = 8.0) -> str:
    import subprocess

    t0 = time.monotonic()
    prefer: list[str] = []
    while time.monotonic() - t0 < timeout_sec:
        try:
            out = subprocess.check_output(
                ["ros2", "topic", "list", "-t"], text=True, timeout=3.0
            )
        except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
            time.sleep(0.3)
            continue
        lines = [ln.strip() for ln in out.splitlines() if ln.strip()]
        for ln in lines:
            parts = ln.split()
            if len(parts) < 2:
                continue
            name, typ = parts[0], parts[1]
            if typ != "sensor_msgs/msg/Image":
                continue
            if name == "/processed_image" or name.endswith("/processed_image"):
                prefer.append(name)
        if prefer:
            if "/processed_image" in prefer:
                return "/processed_image"
            for p in prefer:
                if "image_subscriber_node" in p:
                    return p
            return prefer[0]
        time.sleep(0.3)
    return "/processed_image"


def main() -> int:
    rclpy.init()
    topic = os.environ.get("TRACE_TEST_IMAGE_TOPIC", "").strip()
    if not topic:
        print("等待 trace_calculator 订阅话题出现（processed_image）…", flush=True)
        topic = discover_image_topic()
    print(f"发布图像话题: {topic}", flush=True)

    node = SynthTestNode(topic)
    t_start = time.monotonic()
    run_sec = float(os.environ.get("TRACE_TEST_RUN_SEC", "8"))
    end = t_start + run_sec
    try:
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

    pitches = [p for _, p, _ in node._dbg]
    yaws = [y for _, _, y in node._dbg]
    if len(pitches) < 30:
        print(f"FAIL: tracking_debug 样本过少 n={len(pitches)}（节点是否在跑？话题名是否正确？）", flush=True)
        return 2

    # 丢弃前 1.5s（无目标累帧 + 进入 search_mode）
    t_cut = t_start + 1.5
    late = [(p, y) for t, p, y in node._dbg if t >= t_cut]
    if len(late) < 20:
        late = [(p, y) for _, p, y in node._dbg[-80:]]
    lp = [a for a, _ in late]
    ly = [b for _, b in late]

    def span(xs: list[float]) -> float:
        return max(xs) - min(xs) if xs else 0.0

    ps = span(lp)
    ys = span(ly)
    print(
        f"样本数={len(node._dbg)} 后半段 pitch 极差={ps:.2f}° yaw 极差={ys:.2f}°",
        flush=True,
    )

    # 大能量纯色块搜索应同时动 pitch 与 yaw（极差应明显大于数值噪声）
    ok_pitch = ps >= 2.0
    ok_yaw = ys >= 3.0
    if ok_pitch and ok_yaw:
        print("PASS: 检测到俯仰与偏航搜索摆动（丢目标后进入 search_mode）。", flush=True)
        return 0
    if ok_yaw and not ok_pitch:
        print(
            "WARN: 偏航在扫但俯仰极差偏小；若现场仍「只转不抬头」请检查 large+track_color_square_only 分支。",
            flush=True,
        )
        return 1
    print(
        "FAIL: 舵机角摆动不足（可能未进入搜索、图像未送达、或 GUI/imshow 阻塞）。",
        flush=True,
    )
    return 3


if __name__ == "__main__":
    sys.exit(main())

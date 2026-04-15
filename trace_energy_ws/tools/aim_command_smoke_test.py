#!/usr/bin/env python3
"""
无相机：向 trace_calculator 发灰图 + aim_command，断言 tracking_debug[10]（aim_engaged）随指令变化。

由 run_aim_command_smoke_test.sh 拉起 trace_calculator 后调用；需已 source install/setup.bash。
"""
from __future__ import annotations

import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Int32


def _make_gray_bgr(h: int = 480, w: int = 640) -> bytes:
    b, g, r = 45, 45, 45
    row = bytes([b, g, r] * w)
    return row * h


class AimSmokeNode(Node):
    def __init__(self, image_topic: str) -> None:
        super().__init__("aim_command_smoke_test")
        qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub_img = self.create_publisher(Image, image_topic, qos)
        self._pub_aim = self.create_publisher(Int32, "aim_command", qos)
        self._last_aim_engaged: float | None = None
        self._dbg_count = 0
        self.create_subscription(Float64MultiArray, "/tracking_debug", self._on_dbg, 20)
        self._img = Image()
        self._img.height = 480
        self._img.width = 640
        self._img.encoding = "bgr8"
        self._img.is_bigendian = 0
        self._img.step = 640 * 3
        self._img.data = _make_gray_bgr()
        self._period = 1.0 / 30.0
        self.create_timer(self._period, self._tick_img)

    def _on_dbg(self, msg: Float64MultiArray) -> None:
        self._dbg_count += 1
        if len(msg.data) >= 11:
            self._last_aim_engaged = float(msg.data[10])

    def _tick_img(self) -> None:
        self._img.header.stamp = self.get_clock().now().to_msg()
        self._pub_img.publish(self._img)

    def publish_aim(self, v: int) -> None:
        m = Int32()
        m.data = int(v)
        self._pub_aim.publish(m)
        self.get_logger().info(f"published aim_command data={v}")


def wait_for_debug(node: AimSmokeNode, min_msgs: int, timeout_sec: float) -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_sec and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.15)
        if node._dbg_count >= min_msgs and node._last_aim_engaged is not None:
            return True
    return False


def main() -> int:
    rclpy.init()
    topic = os.environ.get("TRACE_TEST_IMAGE_TOPIC", "/processed_image").strip() or "/processed_image"
    node: AimSmokeNode | None = None
    rc = 99
    try:
        node = AimSmokeNode(topic)
        time.sleep(0.25)
        if not wait_for_debug(node, min_msgs=8, timeout_sec=12.0):
            print("FAIL: 未收到足够 tracking_debug（trace_calculator 是否在跑？）", flush=True)
            rc = 2
            return rc
        # 与其它节点同域残留状态脱钩：先发 0 再等到 dbg[10] 归零
        for _ in range(10):
            node.publish_aim(0)
            time.sleep(0.04)
        t_clear = time.monotonic()
        while time.monotonic() - t_clear < 5.0 and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.12)
            if node._last_aim_engaged is not None and node._last_aim_engaged < 0.5:
                break
        if node._last_aim_engaged is None or node._last_aim_engaged > 0.5:
            print(
                f"FAIL: 清位后 aim_engaged 未回到 0（dbg[10]={node._last_aim_engaged}，检查 ROS_DOMAIN_ID 是否与其它 trace 冲突）",
                flush=True,
            )
            rc = 3
            return rc

        node.publish_aim(1)
        t0 = time.monotonic()
        ok_high = False
        while time.monotonic() - t0 < 6.0 and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.15)
            if node._last_aim_engaged is not None and node._last_aim_engaged > 0.5:
                ok_high = True
                break
        if not ok_high:
            print(
                "FAIL: aim_command=1 后 tracking_debug[10] 未变为 1（Qt/话题逻辑未生效？）",
                flush=True,
            )
            rc = 4
            return rc

        for _ in range(6):
            node.publish_aim(0)
            time.sleep(0.06)
        t0 = time.monotonic()
        ok_low = False
        while time.monotonic() - t0 < 10.0 and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.15)
            if node._last_aim_engaged is not None and node._last_aim_engaged < 0.5:
                ok_low = True
                break
        if not ok_low:
            print("FAIL: aim_command=0 后 dbg[10] 未回到 0", flush=True)
            rc = 5
            return rc

        print(
            "PASS: aim_command 1→dbg[10]=1，0→dbg[10]=0（与 aim_workflow_enabled 无关）",
            flush=True,
        )
        rc = 0
        return rc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

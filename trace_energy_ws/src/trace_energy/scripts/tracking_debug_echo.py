#!/usr/bin/env python3
"""订阅 trace_calculator 发布的 /tracking_debug，终端打印或写入 CSV。"""
import argparse
import csv
import sys
import time
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
except ModuleNotFoundError as exc:
    if "_rclpy" in str(exc) or "rclpy" in str(exc):
        sys.stderr.write(
            "\n[rclpy] 导入失败：请 conda deactivate 后使用 Python 3.10，或：\n"
            "  bash tools/run_with_ros2_python.sh src/trace_energy/scripts/tracking_debug_echo.py\n\n"
        )
    raise SystemExit(2) from exc


class TrackingDebugEcho(Node):
    def __init__(self, csv_path: Optional[str], print_each: bool):
        super().__init__("tracking_debug_echo")
        self._csv_path = csv_path
        self._print_each = print_each
        self._csv_file = None
        self._writer = None
        if csv_path:
            self._csv_file = open(csv_path, "w", newline="", encoding="utf-8")
            self._writer = csv.writer(self._csv_file)
            self._writer.writerow(
                [
                    "t_wall",
                    "hit_error_px",
                    "x_error",
                    "y_error",
                    "detect",
                    "hold",
                    "miss_count",
                    "pitch_deg",
                    "yaw_deg",
                    "fps",
                    "control_enabled",
                    "aim_engaged_large_square",
                    "filt_ex",
                    "u_yaw",
                    "pitch_cmd_i",
                    "yaw_cmd_i",
                    "hw_pitch_id_deg",
                    "hw_yaw_id_deg",
                    "center_ref_id",
                    "target_id",
                    "center_lab_id",
                ]
            )
        self.create_subscription(Float64MultiArray, "/tracking_debug", self._cb, 10)

    def _cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 10:
            self.get_logger().warn(f"short tracking_debug len={len(msg.data)}")
            return
        d = msg.data
        t = time.time()
        aim_e = float(d[10]) if len(d) > 10 else 0.0
        fex = float(d[11]) if len(d) > 11 else 0.0
        u_yaw = float(d[12]) if len(d) > 12 else 0.0
        pc = float(d[13]) if len(d) > 13 else d[6]
        yc = float(d[14]) if len(d) > 14 else d[7]
        hwp = float(d[15]) if len(d) > 15 else pc
        hwy = float(d[16]) if len(d) > 16 else yc
        cref = float(d[17]) if len(d) > 17 else -1.0
        tid = float(d[18]) if len(d) > 18 else -1.0
        clab = float(d[19]) if len(d) > 19 else -1.0
        row = [
            t,
            d[0],
            d[1],
            d[2],
            int(d[3]),
            int(d[4]),
            int(d[5]),
            d[6],
            d[7],
            d[8],
            int(d[9]),
            aim_e,
            fex,
            u_yaw,
            pc,
            yc,
            hwp,
            hwy,
            cref,
            tid,
            clab,
        ]
        if self._writer:
            self._writer.writerow(row)
            self._csv_file.flush()
        if self._print_each:
            ae = f" aim={int(d[10])}" if len(d) > 10 else ""
            fe = f" fex={d[11]:.1f} uy={d[12]:.2f}" if len(d) > 12 else ""
            pcmd = f" P/Yi={int(d[13])}/{int(d[14])} hw={int(d[15])}/{int(d[16])}" if len(d) > 16 else ""
            cid = f" cref={int(d[17])} tgt={int(d[18])} lab={int(d[19])}" if len(d) > 19 else ""
            print(
                f"hit={d[0]:.1f} dx={d[1]:.0f} dy={d[2]:.0f} "
                f"det={int(d[3])} hold={int(d[4])} miss={int(d[5])} "
                f"pitch={d[6]:.1f} yaw={d[7]:.1f} fps={d[8]:.1f} ctrl={int(d[9])}{ae}{fe}{pcmd}{cid}",
                flush=True,
            )

    def destroy_node(self) -> bool:
        if self._csv_file:
            self._csv_file.close()
        return super().destroy_node()


def main() -> None:
    p = argparse.ArgumentParser(description="Echo /tracking_debug to terminal and/or CSV")
    p.add_argument(
        "--csv",
        metavar="PATH",
        help="若指定则写入 CSV（含表头），可与终端打印同时使用",
    )
    p.add_argument(
        "--quiet",
        action="store_true",
        help="不写终端行（仅写 CSV 时建议加）",
    )
    args = p.parse_args()
    if not args.csv and args.quiet:
        print("需要 --csv 或去掉 --quiet", file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    node = TrackingDebugEcho(csv_path=args.csv, print_each=not args.quiet)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

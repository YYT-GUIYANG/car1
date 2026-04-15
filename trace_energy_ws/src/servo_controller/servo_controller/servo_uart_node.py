import os
import serial
import rclpy
from rclpy.node import Node
# 导入自定义消息（包名.消息类型）
from servo_message.msg import ServoMessage

# =========================
# [TUNE] 通信层可调参数总览
# =========================
# 1) baudrate (建议: 115200~460800)
#    - 调大: 传输延时下降，跟手更快
#    - 过大风险: 某些线材/板卡会丢包或乱码
#
# 2) timeout / write_timeout (建议: 0.005~0.03)
#    - 调小: 阻塞更少
#    - 过小风险: 偶发写超时
#
# 3) send_timer 周期(建议: 0.01~0.03 秒，即 100~33Hz)
#    - 调小: 控制更新更快
#    - 过小风险: 串口负载增大、抖动加剧
#
# 4) cmd_callback 日志周期(建议: 0.2~1.0 秒)
#    - 日志过密会拖慢节点
#

# 初始化串口（自动适配USB串口和Jetson串口）
# 与你 Jetson V1.1 一致：默认 9600（可用环境变量 SERVO_BAUDRATE 覆盖）
baudrate = int(os.environ.get("SERVO_BAUDRATE", "9600"))
ser = None
forced_port = os.environ.get("SERVO_PORT", "").strip()
port_candidates = [forced_port] if forced_port else ['/dev/ttyACM1', '/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyTHS1']
for port_name in port_candidates:
    if not port_name:
        continue
    try:
        ser = serial.Serial(
            port=port_name,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01,  # 更短超时，减少阻塞
            write_timeout=0.01,
        )
        print(f"串口初始化成功：{port_name}, baudrate={baudrate}")
        break
    except serial.SerialException as e:
        print(f"串口{port_name}不可用：{e}")

if ser is None:
    print("串口初始化失败：未找到可用串口（可设置 SERVO_PORT=/dev/ttyACM1 强制指定）")
    exit(1)

# 默认关闭：每次 TX 都 print 会严重拖慢 trace帧率（易跌到个位数 FPS）。调试时 export SERVO_DEBUG_TX=1
_SERVO_DEBUG_TX = os.environ.get("SERVO_DEBUG_TX", "").strip().lower() in ("1", "true", "yes")


def _servo_angle_bounds():
    """180° 舵机：export SERVO_ANGLE_MAX=180；360° 位置舵机：默认 0~360（与 trace_calculator 限位一致）"""
    lo = int(os.environ.get("SERVO_ANGLE_MIN", "0"))
    hi = int(os.environ.get("SERVO_ANGLE_MAX", "360"))
    if lo > hi:
        lo, hi = hi, lo
    lo = max(0, min(lo, 999))
    hi = max(0, min(hi, 999))
    return lo, hi


# 舵机控制函数（复用+增加参数校验）
def UARTServo(servonum, angle):
    # 校验参数合理性（根据实际舵机范围调整）
    if not (0 < servonum < 17):
        print(f"无效舵机编号：{servonum}（范围1-16）")
        return
    ang_lo, ang_hi = _servo_angle_bounds()
    try:
        a = int(angle)
    except (TypeError, ValueError):
        print(f"无效舵机角度：{angle}")
        return
    # 360° 位置舵机：负角/360+ 先取模，避免整包拒收导致「话题有数但舵机不动」
    if ang_hi - ang_lo >= 359:
        a = (a % 360 + 360) % 360
    if not (ang_lo <= a <= ang_hi):
        print(f"无效舵机角度：{angle}（范围{ang_lo}-{ang_hi}，或改 SERVO_ANGLE_MIN/MAX）")
        return
    
    servonum = 64 + servonum
    date1 = int(a / 100 + 48)
    date2 = int((a % 100) / 10 + 48)
    date3 = int((a % 10) + 48)
    cmd = bytearray([36, servonum, date1, date2, date3, 35])
    
    try:
        ser.write(cmd)
        if _SERVO_DEBUG_TX:
            print(f"TX servo cmd: id={servonum-64}, angle={angle}, hex={cmd.hex()}")
        # 不再固定 sleep，发送节拍由 ROS 定时器统一控制
    except serial.SerialException as e:
        print(f"指令发送失败：{e}")

# ROS2节点类
class ServoUARTNode(Node):
    def __init__(self):
        # 初始化节点，命名为servo_uart_node
        super().__init__('servo_uart_node')
        # 创建订阅者：订阅servo_control话题，回调处理指令
        self.subscription = self.create_subscription(
            ServoMessage,          # 消息类型（自定义）
            'servo_control',   # 话题名
            self.cmd_callback, # 回调函数
            20                # 消息队列大小（避免双轴快速连续消息被挤掉）
        )
        # 缓存每个舵机的最新目标角度，按固定周期发送，避免回调里阻塞
        self.latest_targets = {}
        self.last_sent = {}
        self._flush_tick = 0
        self.last_recv_log_ns = 0
        # [TUNE] 发送周期：0.02=50Hz（稳定优先）
        # 更快可试 0.015 或 0.01；出现抖动/丢包就回调大
        self.send_timer = self.create_timer(0.02, self.flush_latest_commands)
        self.get_logger().info(f"舵机UART节点已启动（{baudrate}bps, 50Hz发送）")

    # 话题回调函数：接收指令并控制舵机
    def cmd_callback(self, msg):
        servo_num = msg.servo_id  # 从消息中取舵机编号
        angle = msg.servo_angle   # 从消息中取角度
        # 只保留每个舵机最新目标，减少通信排队延迟
        self.latest_targets[servo_num] = angle
        # 降低日志频率，避免刷屏拖慢节点
        now_ns = self.get_clock().now().nanoseconds
        # [TUNE] 日志节流周期：0.5s
        if now_ns - self.last_recv_log_ns > 500_000_000:
            self.last_recv_log_ns = now_ns
            self.get_logger().info(f"接收最新目标：{self.latest_targets}")

    def flush_latest_commands(self):
        if not self.latest_targets:
            return
        self._flush_tick += 1
        # 约每 0.5s 强制重发当前目标角一次（50Hz*25），避免「角度未变不去重发」时部分控制板休眠/丢包后舵机僵死
        force_repeat = self._flush_tick % 25 == 0
        for servo_num, angle in list(self.latest_targets.items()):
            last = self.last_sent.get(servo_num)
            if last == angle and not force_repeat:
                continue
            UARTServo(servo_num, angle)
            self.last_sent[servo_num] = angle

# 主函数
def main(args=None):
    rclpy.init(args=args)          # 初始化ROS2
    node = None
    try:
        node = ServoUARTNode()     # 创建节点
        rclpy.spin(node)           # 保持节点运行（阻塞）
    except KeyboardInterrupt:      # 捕获Ctrl+C
        if node is not None:
            node.get_logger().info("节点被手动终止！")
    finally:
        try:
            if ser is not None and getattr(ser, "is_open", False):
                ser.close()
        except Exception:
            pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        # launch 整体 SIGINT 时上下文可能已由框架 shutdown，避免二次 shutdown 抛 RCLError
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
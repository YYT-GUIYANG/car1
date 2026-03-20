import serial
import time
import rclpy
from rclpy.node import Node
# 导入自定义消息（包名.消息类型）
from servo_message.msg import ServoMessage

# 初始化串口（复用原有逻辑）
try:
    ser = serial.Serial(
        port='/dev/ttyTHS1',    # 串口设备名，根据实际修改
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1  # 添加超时，避免串口阻塞
    )
except serial.SerialException as e:
    print(f"串口初始化失败：{e}")
    exit(1)

# 舵机控制函数（复用+增加参数校验）
def UARTServo(servonum, angle):
    # 校验参数合理性（根据实际舵机范围调整）
    if not (0 < servonum < 17):
        print(f"无效舵机编号：{servonum}（范围1-16）")
        return
    if not (0 <= angle <= 180):
        print(f"无效舵机角度：{angle}（范围0-180）")
        return
    
    servonum = 64 + servonum
    date1 = int(angle / 100 + 48)
    date2 = int((angle % 100) / 10 + 48)
    date3 = int((angle % 10) + 48)
    cmd = bytearray([36, servonum, date1, date2, date3, 35])
    
    try:
        ser.write(cmd)
        time.sleep(0.05)
        print(f"指令发送成功：舵机{servonum-64}，角度{angle}")
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
            10                 # 消息队列大小
        )
        self.get_logger().info("舵机UART节点已启动！等待servo_control话题指令...")

    # 话题回调函数：接收指令并控制舵机
    def cmd_callback(self, msg):
        servo_num = msg.servo_id  # 从消息中取舵机编号
        angle = msg.servo_angle   # 从消息中取角度
        # self.get_logger().info(f"收到指令：舵机{servo_num} → 角度{angle}")
        UARTServo(servo_num, angle)  # 执行舵机控制

# 主函数
def main(args=None):
    rclpy.init(args=args)          # 初始化ROS2
    node = ServoUARTNode()         # 创建节点
    try:
        rclpy.spin(node)           # 保持节点运行（阻塞）
    except KeyboardInterrupt:      # 捕获Ctrl+C
        node.get_logger().info("节点被手动终止！")
    finally:
        ser.close()                # 关闭串口
        node.destroy_node()        # 销毁节点
        rclpy.shutdown()           # 关闭ROS2

if __name__ == '__main__':
    main()
# 云台固定环绕测试与赛前清单

## 一、先做纯控制测试（不依赖视觉）

1. 打开串口权限会话：
   - `newgrp dialout`
2. 启动舵机串口节点：
   - `source /opt/ros/humble/setup.bash`
   - `cd /tmp/ros2_trace_energy_mechanism/trace_energy_ws`
   - `source install/setup.bash`
   - `SERVO_BAUDRATE=9600 ros2 run servo_controller servo_uart_node`
3. 新终端运行环绕测试脚本（小幅）：
   - `source /opt/ros/humble/setup.bash`
   - `cd /home/yangyuting/桌面/yyttest/ros2_trace_energy_mechanism/trace_energy_ws`
   - `python3 tools/servo_orbit_test.py --id-pitch 8 --id-yaw 11 --amp-pitch 8 --amp-yaw 12 --hz 0.18 --duration 20`

## 二、安全默认参数（推荐起步）

- `id_pitch=8`, `id_yaw=11`
- `center_pitch=90`, `center_yaw=90`
- `amp_pitch=8`, `amp_yaw=12`
- `hz=0.18`（慢速，便于看方向）
- `rate=25`
- `duration=20`

放大验证（第二步）：

- `--amp-pitch 14 --amp-yaw 20 --hz 0.25 --duration 30`

## 三、你明天大板测试前建议

1. **机械方向确认**
   - 只动俯仰：`--amp-yaw 0`
   - 只动偏航：`--amp-pitch 0`
2. **边界确认**
   - 逐步增幅，确认无撞限位、无卡滞。
3. **热稳定**
   - 连续跑 2~3 分钟，观察是否掉帧/掉指令。
4. **再回视觉联调**
   - 机械链路稳定后再开启识别与跟随。


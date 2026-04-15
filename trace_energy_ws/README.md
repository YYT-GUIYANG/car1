cd /home/yangyuting/桌面/yyttest/ros2_trace_energy_mechanism/trace_energy_ws
bash go.sh
修改版
bash tools/repair_servo_message_and_build.sh
source install/setup.bash
colcon build --packages-select trace_energy --symlink-install

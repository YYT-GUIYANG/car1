# 程序启动注意事项
要启动整个追踪系统，需要执行以下命令：
```bash
source ~/Desktop/program_files/vscode_files/ros2_trace_energy_mechanism/start_trace.sh
```

在终端会看到很多类似指令输出
```bash
[trace_calculator-2] [INFO] [1773961771.710393674] [image_subscriber_node]: 接收帧率：5.0 FPS | 图像分辨率：1920x1080
[trace_calculator-2] [INFO] [1773961771.713791981] [image_subscriber_node]: 找到一个圆形轮廓，9.53, 面积：116.00, 周长：50.63,圆心坐标点位置(152,145)
[trace_calculator-2] [INFO] [1773961771.715472141] [image_subscriber_node]: 发布舵机指令 → ID：1，角度：0
[trace_calculator-2] [INFO] [1773961771.715578863] [image_subscriber_node]: 发布舵机指令 → ID：10，角度：0
```

- 接收帧率低暂时先不用管，估计是因为我是用ros2启动的摄像头画面发布，你先别去动这块，先把图像处理和追踪算法写好
- 只要终端输出发布舵机指令，说明云台舵机控制系统可以正常运行，你可以继续完善追踪算法
- 圆形轮廓找的有很大问题，需要你重新完善一下

**你要写的部分在`trace_energy_ws/src/trace_energy/src/trace_calculator.cpp`文件中，我用`TODO`标记了你需要填写代码的位置，其他地方的代码不要修改。在代码文件中按`ctrl+f`，输入`TODO`按下`enter`键，即可跳转到需要填写代码的位置**
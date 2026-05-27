# ros2_trace_energy_mechanism

基于 **ROS 2 Humble** 的能量机关视觉跟踪与云台控制：从图像里找到中心圆和同色扇区，算出瞄准误差，再通过串口驱动俯仰/偏航舵机。

## 仓库结构（顶层）

```
ros2_trace_energy_mechanism/
├── README.md
├── trace_energy_ws/          ← ROS 工作区
│   ├── src/                  ← 五个功能包
│   ├── go.sh                 ← 一键编译 + 启动
│   ├── clean.sh
│   └── tools/
├── 调试参数/
│   ├── offline_trace.py
│   └── 运行说明.txt
└── start_trace.sh
```

## 各包做什么（简表）


| 包                      | 作用                                                      |
| ---------------------- | ------------------------------------------------------- |
| `**servo_message**`    | 定义 `ServoMessage.msg`，无运行节点                             |
| `**servo_controller**` | `servo_uart_node`，话题转串口（约 50Hz）                         |
| `**trace_energy**`     | `read_image` + `trace_calculator` + 预览 / Qt 工具（**主链路**） |
| `**pidtest`**（可选）      | 通用色块 + PID，复用 `read_image`，**不启** `trace_calculator`    |
| `**simple_track`**（可选） | `simple_blob_vision` + `simple_pd_servo`，最小对照链          |
| 阅读优先级：                 |                                                         |


1. `trace_energy/src/trace_calculator.cpp`
2. `trace_energy/launch/trace_start.launch.py`
3. `调试参数/offline_trace.py`

编译依赖：先编 `servo_message`，再编 `trace_energy` / `pidtest` / `stm32_simple_track`；`servo_controller` 为 Python 包，运行时依赖消息类型。

## 整体架构（简图）

USB 相机 → read_image → /processed_image → trace_calculator → /servo_control → servo_uart_node → 串口 → 舵机板
                                                ├── /trace_debug_image
                                                ├── /tracking_debug
                                                └── /aim_command（可选 Qt 面板）
离线：调试参数/offline_trace.py（同一识别逻辑，无 ROS）


| 能量模式 | 启动参数                 |
| ---- | -------------------- |
| 大能量  | `energy_mode:=large` |
| 小能量  | `energy_mode:=small` |


## 编译与运行

**环境**：Ubuntu 22.04 + ROS 2 Humble + OpenCV；工作区路径建议纯英文；Conda 下先 `conda deactivate`。

```bash
cd /path/to/ros2_trace_energy_mechanism/trace_energy_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select servo_message servo_controller trace_energy --symlink-install
source install/setup.bash
```

**一键启动（主链路）**：

```bash
cd trace_energy_ws && bash go.sh
# 或手动：
ros2 launch trace_energy trace_start.launch.py energy_mode:=large
# 小能量：energy_mode:=small
# 无桌面：show_vis_window:=false
```

## 🧠 实现思路

### 1. 我先做了一个离线识别：先确保能够准确识别目标

  整体实现比较复杂。如果识别和控制同时乱改，很难判断问题出在 Lab 阈值还是 `kp`。所以我先用 `**调试参数/offline_trace.py**` 在摄像头或录像上把识别调稳，再进 ROS。
离线阶段只关心：中心圆和扇区的识别能不能稳定的识别，不去在意舵机。
离线每一帧的识别逻辑（与在线 `trace_calculator` 同源），具体滑条用法、参数含义详见同级目录下调试参数/运行说明.txt。调好后按 `**k**` 保存 生成`offline_trace_params_*.txt`。

### 2. 在线拆成三层：感知 → 解算 → 执行

离线参数满意后，用 ROS 把链路跑起来。我将整个链路拆成三个独立进程，用固定话题连接：

```
read_image  ──/processed_image──►  trace_calculator  ──/servo_control──►  servo_uart_node
  (感知)              (解算)                    (执行)
```

- `**read_image**`（`trace_energy` 包）：打开 USB 相机，发布 `/processed_image`。  
好处：换相机、定曝光不用改算法代码；相机异常不会拖死解算进程。
- `**trace_calculator**`（`trace_energy` 包）：订阅图像 → 识别（和离线四步一样）→ 算 `x_error` / `y_error` → PD 发角度。  
这是主改文件，算法和参数都在这里。
- `**servo_uart_node**`（`servo_controller` 包）：订阅 `ServoMessage`，缓存每个舵机的最新角度，以约 **50Hz** 写串口。  
好处：改串口协议、波特率不用动 C++视觉。
还有一个 `**servo_message`** 包，只定义 `ServoMessage`（`servo_id` + `servo_angle` + 时间戳），让 C++ 和 Python 共用同一数据类型。话题名约定为 `/servo_control`。
节点之间**不互相 include**，只通过话题通信。可以三终端分别起采图、解算、串口，也可以用 `**trace_start.launch.py`** 一键拉起。

### 3.trace_calculator 里面干了什么

在线解算节点其实就是把离线那四步拿过来，然后在后面加上瞄准和控制：
/processed_image
  → 预处理（切ROI、调Gamma、CLAHE、缩宽度）
  → Lab分割 → 拿到一堆轮廓
  → 挑中心圆（看圆度、面积比、离画面中心近不近）
  → HSV精确匹配颜色 → 得到 target_color_id
  → 挑扇区（颜色相同、顶点4~7个、面积和距离合适，取最大的）
  → 算 x_error, y_error（目标中心 vs 激光准星）
  → PD + lead → 算出俯仰/偏航角度 → 发 ServoMessage
  → 同时发布调试图 /trace_debug_image 和误差数组 /tracking_debug
大能量和小能量不是两套代码，而是同一个节点里用 energy_mode:=large 或 small 切换参数：
    大能量：用 large_lab_thresh，预测幅度大一些，搜索摆幅也大。
    小能量：用 small_lab_thresh，锁中心色，丢检后做正弦搜索。
  `control_enabled:=false` 时只画框不发舵机，方便在线只验证识别。
**验收指标**：`hit_error_px = sqrt(x_error² + y_error²)` 能下降并稳定；调试图上中心圆和扇区框合理。

### 4. 调参顺序：A → B → C → D

我调参数的顺序（A→B→C→D）
调多了就发现，按这个顺序来最不容易乱。
    A 识别：先保证中心圆和扇区大多数帧都能找到。主要改 lab_thresh、min_area、ROI比例、圆度阈值、HSV距离上限。标准：静止时误差不乱跳，Mask 干净。
    B 跟手：识别稳了再调控制。主要改 kp、kd、lead_ms、死区。标准：hit_error_px 能降下来，不严重过冲。
    C 丢检恢复：处理短暂遮挡。主要改 coast_*、预测帧数、搜索幅值。标准：遮挡后能找回来，或者至少不乱摆。
    D 通信：最后微调串口。主要改波特率、发送周期（默认50Hz）。标准：指令不丢，云台不一顿一顿。

### 5. 离线参数迁到在线 launch

```
离线调参时按 k 保存的 offline_trace_params_*.txt 文件里，有等价命令行和 JSON。你把对应的值抄到 trace_energy_ws/src/trace_energy/launch/trace_start.launch.py 的默认值里（或者启动时用参数覆盖）。
```

离线参数名和在线不完全一样，但意思一一对应，比如：
    lab_thresh → large_lab_thresh（大能量）或 small_lab_thresh（小能量）
    roi_frac → large_square_roi_frac
    gamma、clahe → vis_input_gamma、clahe_l_clip_limit
    center_circ_min → center_circularity_min
    proc_width_cap → trace_proc_width_cap_large 或 _small
完整的对照表在 调试参数/运行说明.txt 第七节。搬完参数后，执行 bash trace_energy_ws/go.sh 做实车验证。
###三条可选链路（我的调试教训）
比赛主链路就是：read_image → trace_calculator → servo_uart_node。识别中心圆+同色扇区，带预测/搜索状态机。
但调试过程中我发现两个问题：
    1.识别稳了，舵机却跟不好
       离线调参数时，offline_trace.py 已经能把中心圆和扇区框得死死的，可是一上 ROS，云台要么过冲、要么跟不上。这时候很难判断是 kp/kd 不对，还是 lead_ms 有问题，还是相机曝光抖。所以我写了 pidtest 包：它去掉“中心圆+扇区”逻辑，只做单一颜色方块的通用跟踪 + PID。这样我就可以在识别确定的情况下，单独调 kp、kd、死区、行程限位，不用管机关的状态机。
    启动：bash trace_energy_ws/start_pidtest_visual.sh

```
2.但是调试到后面，发现实现效果还是不行，其实这里就已经崩溃了，怀疑主链路写太重了
有时参数都试了一圈，云台还是怪怪的——反应慢、偶尔抽风。我开始怀疑是不是 trace_calculator 里的预测、搜索、丢检保持这些逻辑太重，反而干扰了最基本的“看见就追”。于是又弄了个 simple_track 包：极简版 —— simple_blob_vision 只输出目标坐标，simple_pd_servo 单独做 PD，没有状态机、没有模式切换。
说实话，这个包我最后没来得及实车验证，但它的代码结构留下来，以后如果主链路再出玄学问题，可以用来做 baseline 对照。
启动：ros2 launch simple_track simple_track_full.launch.py
注：同一时间只能跑一条链路，别同时 launch 两个。
```

【主链路 / pidtest】  结构类似，只是换了解算节点名
read_image → /processed_image → trace_calculator 或 pidtest_node → /servo_control → servo_uart_node
【simple简化】  视觉与控制拆成两个进程
read_image → /processed_image → simple_blob_vision → /blob_xy → simple_pd_servo → /servo_control → servo_uart_node
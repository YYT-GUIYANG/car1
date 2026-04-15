/**
 * 大能量瞄准 Qt面板：与 trace_calculator 的 /aim_command (Int32) 配合。
 * 流程：形状定位中心圆 → 读中心色 → 周围同色扇区追踪（trace内实现）。
 * 0=停止瞄准 1=开始瞄准  2=重新锁定（对当前帧中心圆读色并锁定 id）
 */
#include <QApplication>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("aim_control_panel");
    auto pub = node->create_publisher<std_msgs::msg::Int32>(
        "/aim_command", rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

    auto send = [pub](int v) {
        std_msgs::msg::Int32 m;
        m.data = v;
        pub->publish(m);
    };

    QMainWindow win;
    win.setWindowTitle("能量机关瞄准 — Qt 面板");
    win.resize(520, 320);

    auto* central = new QWidget(&win);
    win.setCentralWidget(central);
    auto* v = new QVBoxLayout(central);

    auto* hint = new QLabel(
        "<b>追踪逻辑（trace_calculator）</b><br/>"
        "1）用<strong>形状</strong>在画面中心附近找<strong>中心圆</strong>；<br/>"
        "2）对中心圆轮廓内<strong>均值颜色</strong>做 HSV 与 color_table 比对，得到要追的 <strong>id</strong>；<br/>"
        "3）在转盘周围找<strong>同色扇区块</strong>并持续跟踪（紫系 id0/3/6 可走族匹配）。<br/>"
        "<br/>"
        "<b>红外/激光</b>：默认在光轴中心<strong>下方</strong>约 4cm，已用 "
        "<code>laser_aim_offset_y_px</code>（launch 默认 34px）参与瞄准；请对照光斑微调。<br/>"
        "<br/>"
        "舵机映射：<b>俯仰→8 号</b>、<b>偏航→11 号</b>（<code>swap_pitch_yaw_channels:=false</code>）。<br/>"
        "画面见 OpenCV 窗口「Received Image」。话题 <code>/aim_command</code>："
        "<b>0</b>=停 <b>1</b>=开始（工作流开时若未锁色会先读本帧中心圆再追） <b>2</b>=仅重锁中心色不追",
        &win);
    hint->setWordWrap(true);
    hint->setTextFormat(Qt::RichText);
    v->addWidget(hint);

    auto* sep = new QFrame(&win);
    sep->setFrameShape(QFrame::HLine);
    v->addWidget(sep);

    auto* row = new QHBoxLayout();
    auto* b2 = new QPushButton("② 重锁中心圆颜色");
    auto* b1 = new QPushButton("① 开始瞄准");
    auto* b0 = new QPushButton("停止");
    b2->setToolTip("对当前帧「形状找到的中心圆」读 HSV 色 id并锁定，再点开始");
    b1->setToolTip("持续追踪与中心同色的扇区块");
    b0->setToolTip("停止发舵机瞄准增量（话题发 0）");
    QObject::connect(b2, &QPushButton::clicked, [&send]() { send(2); });
    QObject::connect(b1, &QPushButton::clicked, [&send]() { send(1); });
    QObject::connect(b0, &QPushButton::clicked, [&send]() { send(0); });
    row->addWidget(b2);
    row->addWidget(b1);
    row->addWidget(b0);
    v->addLayout(row);

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [node]() { rclcpp::spin_some(node); });
    timer.start(40);

    win.show();
    const int rc = app.exec();
    rclcpp::shutdown();
    return rc;
}

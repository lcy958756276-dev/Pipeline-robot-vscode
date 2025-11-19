/**
 * @file ros_tcp_sender.cpp
 * @brief 封装 TCP Socket 与 ROS 通信发送功能
 *
 * 功能：
 * - 使用 Qt QTcpSocket 建立 TCP 连接
 * - 向指定 IP/端口发送 ROS 指令数据
 * - 处理 TCP 错误
 */

#include "ros_tcp_sender.h"
#include "ros/ros.h"

/**
 * @brief 构造函数，初始化 TCP Socket 并连接错误信号
 * @param parent QObject 父对象
 */
RosTcpSender(QObject *parent) : QObject(parent), tcpSocket_(new QTcpSocket(this)) {
    // Qt 5.12 兼容的连接方式
    connect(tcpSocket_, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::error),
            this, &RosTcpSender::onTcpError);
}

/**
 * @brief 发送 ROS 指令数据
 * @param data 要发送的 QByteArray 数据
 *
 * 如果当前 TCP Socket 未连接，会尝试连接到 192.168.8.11:12345。
 * 连接成功后写入数据并等待写入完成。
 */
void sendCommand(const QByteArray &data) {
    if (tcpSocket_->state() == QAbstractSocket::UnconnectedState) {
        tcpSocket_->connectToHost("192.168.8.11", 12345);
        ROS_INFO("88");
    } else {
        if (tcpSocket_->waitForConnected(3000)) {
            tcpSocket_->write(data);
            ROS_INFO("99");
            tcpSocket_->waitForBytesWritten(1000);
        }
    }
}

/**
 * @brief TCP 错误处理槽函数
 * @param error QAbstractSocket::SocketError 错误类型
 *
 * 当 TCP 出现错误时打印错误信息到控制台。
 */
void onTcpError(QAbstractSocket::SocketError error) {
    qDebug() << "TCP Error:" << tcpSocket_->errorString();
}

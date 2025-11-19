/**
 * @file serial_readwrite_node.cpp
 * @brief ROS节点，用于通过串口与STM32通信，实现数据接收和发送
 *
 * 功能：
 * - 订阅 "cable" 主题，将接收到的字符串通过串口发送给STM32。
 * - 接收STM32返回的数据，发布到 "minipc_echo" 主题。
 * - 支持对特定命令（如 "AC" 开头）进行数值处理后再发送。
 */

#include <ros/ros.h>
#include <serial/serial.h> // ROS 内置串口库
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt8MultiArray.h"
#include <cstdlib>
#include <string>
#include <stdexcept>
#include <cmath>

/** @brief 全局串口对象 */
serial::Serial ser;

/**
 * @brief 处理订阅到的发送数据，将其发送到串口
 * @param msg std_msgs::String 发送的字符串消息
 *
 * 功能：
 * - 如果消息以 "AC" 开头，将消息中的数字解析出来，进行特定计算后发送。
 * - 否则直接将消息追加 "\r\n" 发送到串口。
 */
void datasend_callback(const std_msgs::String::ConstPtr &msg)
{
    int len = msg->data.size();
    ROS_INFO("Successfully get: %s", msg->data.c_str());

    // 特殊处理 "AC" 开头的消息
    if (msg->data.substr(0, 2) == "AC")
    {
        double number = std::stod(msg->data.substr(3));
        // 根据公式处理数字
        double newNumber = 72000000 / (number * 1200 * 3000 / 0.17) - 1;
        std::string output = "AC+" + std::to_string(newNumber);
        std::string command_to_send = output + "\r\n";
        ROS_INFO("Successfully send: %s", command_to_send.c_str());
        ser.write(command_to_send);
        return;
    }

    // 普通字符串直接发送
    std::string command_to_send = msg->data + "\r\n";
    ser.write(command_to_send);
}

/**
 * @brief 主函数，初始化节点，打开串口，订阅/发布主题
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出状态
 *
 * 功能：
 * - 初始化 ROS 节点。
 * - 打开串口 "/dev/ttyACM0"，波特率 19200。
 * - 订阅 "cable" 主题发送数据。
 * - 发布 "minipc_echo" 主题接收数据。
 * - 循环读取串口数据并发布。
 */
int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "serial_readwrite_node");
    ros::NodeHandle nh;

    static int len;
    std_msgs::String RecvData;

    // 订阅发送主题
    ros::Subscriber send_sub = nh.subscribe("cable", 1000, datasend_callback);
    // 发布接收主题
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("minipc_echo", 1000);

    try
    {
        // 设置串口属性并打开
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(19200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();

        // 使用 stty 设置串口参数
        std::string port = "/dev/ttyACM0";
        std::string command = "stty -F " + port + " cs8 -cstopb -parenb";
        if (system(command.c_str()) != 0)
        {
            ROS_ERROR_STREAM("Failed to set serial port parameters using stty");
            return -1;
        }
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port /dev/ttyACM0");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(10); // 循环频率 10Hz

    while (ros::ok())
    {
        len = ser.available();

        if (len > 2)
        {
            std_msgs::UInt8MultiArray serial_data;

            // 读取串口数据
            ser.read(serial_data.data, len);

            RecvData.data.clear(); // 清除上一次数据

            for (size_t i = 0; i < len; i++)
            {
                RecvData.data.push_back(serial_data.data[i]);
            }

            ROS_INFO("Successfully received: %s", RecvData.data.c_str());

            // 发布接收到的数据
            if (RecvData.data.size() > 1)
            {
                read_pub.publish(RecvData);
            }
        }

        // 处理订阅消息
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/**
 * @file odometry_publisher.cpp
 * @brief 基于轮速编码器计算里程计并发布 /cable/odom，同时广播 tf 变换
 *
 * 功能：
 * - 订阅 `/minipc_echo` 获取轮速编码器脉冲。
 * - 根据轮半径和脉冲计算机器人行进距离。
 * - 发布 `/cable/odom` 里程计消息。
 * - 广播 `odom -> base_link` 的 tf 变换。
 * - 支持通过 `/reset_odom` 重置里程计。
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <regex>
#include <std_msgs/Bool.h>

/** @brief 全局变量 */

// 重置里程计标志
bool reset_odom = false;

// 轮子参数
double wheel_radius = 0.0318;  ///< 轮子半径(m)
double wheel_base = 0.343;     ///< 轮距(m)

// 行进距离
double distance = 0.0;
// double right_distance = 0.0; // 右轮距离（未使用）

// 当前位姿
double x = 0.0;
double y = 0.0;
double theta = 0.0;

// 每脉冲对应的角度
double pluse = 2.295048561 * 360.0 / (400*4);

// 上一次更新时间
ros::Time last_time;

/**
 * @brief 从编码器字符串消息中提取左轮增量数
 * @param input 编码器字符串消息
 * @return std::string 左轮增量字符串
 */
std::string extra(const std::string& input)
{
    std::string prefix="e1:";
    std::string num;
    std::size_t prefix_pos = input.find(prefix);
    for(int i = prefix_pos+3; i < input.size(); i++)
    {
        if(input[i] == 'p') break;
        num += input[i];
    }
    return num;
}

/**
 * @brief 接收 /reset_odom 消息，设置重置标志
 * @param msg std_msgs::Bool 消息，true 表示重置
 */
void resetOdometryCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data == true)
    {
        reset_odom = true; // 设置重置标志为真
    }
}

/**
 * @brief 接收编码器字符串消息，计算左轮行进距离
 * @param msg std_msgs::String 消息
 */
void EncoderCallback(const std_msgs::String::ConstPtr &msg)
{
    std::string str = msg->data;
    if (str.find("e1") == std::string::npos)
    {
        return;
    }
    
    std::string num_str = extra(str);
    if(num_str.size() > 0)
    {
        int left_increment = std::stoi(num_str);
        // 根据脉冲数计算左轮行进距离
        distance += (left_increment * (pluse / 360.0) * 2 * M_PI * wheel_radius);
    }
}

/**
 * @brief 发布里程计消息并广播 tf 变换
 * @param odom_pub ROS 发布器，发布 nav_msgs::Odometry
 * @param tf_broadcaster tf 广播器
 */
void publishOdometry(ros::Publisher& odom_pub, tf::TransformBroadcaster& tf_broadcaster)
{
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();

    // 处理里程计重置
    if (reset_odom)
    {
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        reset_odom = false; // 重置完成
    }

    double d_distance = distance; // 计算行进距离
    x += d_distance;              // 更新 x 坐标
    // y += d_distance * sin(theta); // 更新 y 坐标（未使用）
    theta = 0;                     // 更新角度

    // 创建里程计消息
    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = d_distance / dt;

    // 发布里程计
    odom_pub.publish(odom);

    // 广播 tf 变换
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    transform.setRotation(q);
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    // 重置行进距离
    distance = 0.0;
    last_time = current_time;
}

/**
 * @brief 主函数，初始化 ROS 节点，订阅编码器和重置指令，发布里程计
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出状态
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle nh;

    // 订阅编码器数据
    ros::Subscriber ticks_sub = nh.subscribe("/minipc_echo", 1000, EncoderCallback);
    // 订阅重置指令
    ros::Subscriber reset_sub = nh.subscribe("/reset_odom", 10, resetOdometryCallback);

    // 发布里程计
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/cable/odom", 1);

    // tf 广播器
    tf::TransformBroadcaster tf_broadcaster;

    // 初始化循环频率
    ros::Rate loop_rate(10); // 10Hz

    while (ros::ok())
    {
        // 发布里程计消息和 tf
        publishOdometry(odom_pub, tf_broadcaster);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

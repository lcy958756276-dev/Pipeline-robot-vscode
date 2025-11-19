/**
 * @file waypoint_follower.cpp
 * @brief ROS 节点，用于接收 waypoint 并控制机器人运动到目标点
 *
 * 该节点功能：
 * - 订阅目标点（waypoint）
 * - 订阅机器人当前位置（odom）
 * - 订阅停止信号（whether_stop）
 * - 发布机器人状态（robot_status）
 * - 发布控制命令到 STM（Pub2stm）
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include <cmath>
#include <thread>
#include <chrono>

/**
 * @class WaypointFollower
 * @brief 负责接收目标点并控制机器人运动
 *
 * 功能：
 * - 接收 waypoint 消息
 * - 判断机器人当前位置与目标点偏差
 * - 发送前进、后退、左右矫正和停止指令
 * - 根据偏差判断是否到达目标点并发布状态
 */
class WaypointFollower
{
public:
    /**
     * @brief 构造函数，初始化订阅者和发布者
     */
    WaypointFollower()
    {
        waypoint_sub = nh.subscribe("waypoint", 10, &WaypointFollower::waypointCallback, this);
        odom_sub = nh.subscribe("/cable/odom", 10, &WaypointFollower::odomCallback, this);
        stop_sub = nh.subscribe("whether_stop", 10, &WaypointFollower::stopCallback, this);
        // cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        status_pub = nh.advertise<std_msgs::Bool>("robot_status", 10);
        control_pub = nh.advertise<std_msgs::String>("Pub2stm", 10);
    }

    /**
     * @brief 析构函数，发送停止指令
     */
    ~WaypointFollower()
    {
        // 析构函数中发布ST消息
        std_msgs::String msg;
        msg.data = "ST";
        control_pub.publish(msg);
        control_pub.publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        control_pub.publish(msg);
    }

    /**
     * @brief waypoint 回调函数
     * @param msg 接收到的目标点 geometry_msgs::PoseStamped 消息
     *
     * 功能：
     * - 更新目标点 X 坐标
     * - 重置到达状态
     * - 标记已接收目标点
     */
    void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        target_position = msg->pose.position.x;
        is_target_reached = false;
        target_position_received = true;
        ROS_INFO("已接收到目标点 X:%f,Y:%f", msg->pose.position.x, msg->pose.position.y);
    }

    /**
     * @brief 停止回调函数
     * @param msg 接收到的停止信号 std_msgs::Bool 消息
     *
     * 功能：
     * - 更新停止标志
     * - 如果接收到停止指令，则打印日志
     */
    void stopCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        stop = msg->data;
        if (stop)
        {
            ROS_INFO("停止自动导航");
        }
    }

    /**
     * @brief Y 方向位置矫正函数
     * @param y_error 当前 Y 坐标偏差
     *
     * 功能：
     * - 根据 Y 偏差发送左右移动和前进指令以矫正位置
     */
    void correctYPosition(double y_error)
    {
        if(y_error < -0.05)
        {
            std_msgs::String msg;
            msg.data="LE";
            control_pub.publish(msg);
            control_pub.publish(msg);
            control_pub.publish(msg);
            control_pub.publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            msg.data="FO";
            control_pub.publish(msg);
            control_pub.publish(msg);
            control_pub.publish(msg);
            control_pub.publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            msg.data="RI";
            control_pub.publish(msg);
            control_pub.publish(msg);
            control_pub.publish(msg);
            control_pub.publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            msg.data="ST";
            control_pub.publish(msg);
            control_pub.publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            control_pub.publish(msg);
        }
        else if(y_error >0.05)
        {
            std_msgs::String msg;
            msg.data="RI";
            control_pub.publish(msg);
            control_pub.publish(msg);
            control_pub.publish(msg);
            control_pub.publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            msg.data="FO";
            control_pub.publish(msg);
            control_pub.publish(msg);
            control_pub.publish(msg);
            control_pub.publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            msg.data="LE";
            control_pub.publish(msg);
            control_pub.publish(msg);
            control_pub.publish(msg);
            control_pub.publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            msg.data="ST";
            control_pub.publish(msg);
            control_pub.publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            control_pub.publish(msg);
        }
    }

    /**
     * @brief odom 回调函数
     * @param msg 接收到的机器人位姿 nav_msgs::Odometry 消息
     *
     * 功能：
     * - 计算机器人当前位置和 Y 偏差
     * - 根据 X 坐标偏差发送前进/后退控制指令
     * - 调用 checkTargetReached 判断是否到达目标
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        double current_position = msg->pose.pose.position.x;
        double current_position_y = msg->pose.pose.position.y;
        double vel=msg->twist.twist.linear.x;

        double y_error = current_position_y - 0.0; // 假设目标y位置是0

        if (!is_target_reached && target_position_received && !stop)
        {
            double error = target_position - current_position;
            
            std_msgs::String msg;
            if (error > 0.20)
            {
                msg.data = "FO";
                ROS_INFO("send         fo\n");
                ROS_INFO("vel=%f",abs(vel));
                control_pub.publish(msg);
            }
            else if (error < -0.20 && abs(vel) < 0.01)
            {
                msg.data = "BA";
                ROS_INFO("send BA");
                control_pub.publish(msg);
            }
            
            checkTargetReached(error, vel);
        }
    }

    /**
     * @brief 检查机器人是否到达目标点
     * @param error X 方向偏差
     * @param vel 当前速度
     *
     * 功能：
     * - 如果偏差在容差范围内且速度大于阈值，则标记已到达
     * - 发布机器人状态
     * - 发布停止指令
     */
    void checkTargetReached(double error,double vel)
    {
        std_msgs::Bool status_msg;
        if (std::abs(error) < tolerance && abs(vel) > 0.04)
        {
            is_target_reached = true;
            status_msg.data = true;
            status_pub.publish(status_msg);
            std_msgs::String msg;
            msg.data = "ST";
            control_pub.publish(msg);
            ROS_INFO("已到达目标点");
        }
        else
        {
            status_msg.data = false;
            status_pub.publish(status_msg);
        }
    }

private:
    ros::NodeHandle nh;                 ///< ROS 节点句柄
    ros::Subscriber waypoint_sub;       ///< waypoint 订阅者
    ros::Subscriber odom_sub;           ///< odom 订阅者
    ros::Subscriber stop_sub;           ///< 停止信号订阅者
    ros::Publisher cmd_vel_pub;         ///< 控制命令发布者
    ros::Publisher status_pub;          ///< 机器人状态发布者
    ros::Publisher control_pub;         ///< STM 控制命令发布者

    double target_position = 0.0;       ///< 当前目标 X 坐标
    bool target_position_received = false; ///< 是否接收到目标点
    bool is_target_reached = false;     ///< 是否到达目标点
    bool stop = false;                  ///< 是否收到停止信号
    const double tolerance = 0.2;      ///< 偏差容差
};

/**
 * @brief 主函数
 * @param argc 参数个数
 * @param argv 参数数组
 * @return int 返回值
 *
 * 功能：
 * - 初始化 ROS 节点
 * - 创建 WaypointFollower 对象
 * - 调用 ros::spin() 保持节点运行
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_follower");
    setlocale(LC_ALL, "");
    WaypointFollower wf;
    ros::spin();
    return 0;
}

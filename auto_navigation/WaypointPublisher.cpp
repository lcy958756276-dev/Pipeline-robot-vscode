/**
 * @file waypoint_publisher.cpp
 * @brief ROS 节点，用于发布机器人导航的中途目标点和最终目标点
 *
 * 该文件实现了一个 ROS 节点 WaypointPublisher，功能如下：
 * - 接收 midgoal 中途目标点并排序发布
 * - 接收 finalgoal 最终目标点并发布
 * - 根据机器人当前位置和状态发布 waypoint
 * - 发布自动导航停止信号 (/whether_auto_navigation)
 * - 接收机器人状态更新，判断目标点是否到达
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <limits>

/**
 * @class WaypointPublisher
 * @brief 中途目标点与最终目标点发布器
 *
 * 功能：
 * - 订阅 midgoal、finalgoal 和机器人状态
 * - 根据机器人当前位置动态发布 waypoint
 * - 管理中途目标队列，并保证按 X 坐标升序发布
 * - 在到达最终目标时发布停止信号
 */
class WaypointPublisher
{
public:
    /**
     * @brief 构造函数，初始化订阅者和发布者
     *
     * 初始化发布者：
     * - waypoint: 发布机器人当前目标点
     * - /whether_auto_navigation: 发布自动导航停止信号
     *
     * 初始化订阅者：
     * - midgoal: 中途目标点
     * - finalgoal: 最终目标点
     * - robot_status: 机器人是否到达目标点
     * - /cable/odom: 获取机器人当前位置
     */
    WaypointPublisher()
    {
        waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("waypoint", 10);
        auto_navigation_stop_pub = nh.advertise<std_msgs::Bool>("/whether_auto_navigation", 10);
        midgoal_sub = nh.subscribe("midgoal", 10, &WaypointPublisher::midgoalCallback, this);
        finalgoal_sub = nh.subscribe("finalgoal", 10, &WaypointPublisher::finalgoalCallback, this);
        status_sub = nh.subscribe("robot_status", 10, &WaypointPublisher::statusCallback, this);
        odom_sub = nh.subscribe("/cable/odom", 10, &WaypointPublisher::odomCallback, this);
        finalgoal.pose.position.x = std::numeric_limits<float>::max();
        pre_goal.pose.position.x = -1;
    }

    /**
     * @brief midgoal 回调函数
     * @param msg 接收到的中途目标点 geometry_msgs::PoseStamped 消息
     *
     * 功能：
     * - 将接收到的中途目标点加入队列
     * - 输出日志
     */
    void midgoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        midgoals.push(*msg);
        ROS_INFO("已加载中途目标点: X:%f,Y:%f", msg->pose.position.x, msg->pose.position.y);
    }

    /**
     * @brief 将队列中的 PoseStamped 按 X 坐标升序排序
     * @param msgQueue 待排序队列
     * @return 排序后的队列
     */
    std::queue<geometry_msgs::PoseStamped> queueSort(std::queue<geometry_msgs::PoseStamped> &msgQueue)
    {
        std::vector<geometry_msgs::PoseStamped> tempVec;

        // 将队列中的元素提取到 vector 中
        while (!msgQueue.empty()) {
            tempVec.push_back(msgQueue.front());
            msgQueue.pop();
        }

        // 对 vector 中的元素进行排序，根据 x 坐标升序排序
        std::sort(tempVec.begin(), tempVec.end(), [](const auto& a, const auto& b) {
            return a.pose.position.x < b.pose.position.x;
        });

        // 将排序后的元素重新放回队列中
        for (const auto& pose : tempVec) {
            msgQueue.push(pose);
        }

        return msgQueue;
    }

    /**
     * @brief odom 回调函数
     * @param msg 接收到的机器人位姿 nav_msgs::Odometry 消息
     *
     * 功能：
     * - 更新当前机器人位置和速度
     * - 判断是否到达最终目标
     * - 发布自动导航停止信号
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        current_position = msg->pose.pose.position.x;
        current_status = msg->twist.twist.linear.x;
        if (abs(current_position - finalgoal.pose.position.x) < 0.3)
        {
            std_msgs::Bool msg;
            msg.data = false;
            finalinital = true;
            auto_navigation_stop_pub.publish(msg);
            finalgoal_reached = true;
        }
        else
        {
            finalgoal_reached = false;
        }
    }

    /**
     * @brief finalgoal 回调函数
     * @param msg 接收到的最终目标点 geometry_msgs::PoseStamped 消息
     *
     * 功能：
     * - 更新 finalgoal 成员变量
     * - 标记最终目标已接收
     * - 如果 finalinital 为 true，则立即发布最终目标点
     */
    void finalgoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        finalgoal = *msg;
        finalgoal_received = true;

        ROS_INFO("已加载最终目标点: X:%f,Y:%f", finalgoal.pose.position.x, finalgoal.pose.position.y);

        if (finalinital)
        {
            waypoint_pub.publish(finalgoal);
            waypoint_pub.publish(finalgoal);
            waypoint_pub.publish(finalgoal);
            pre_goal = finalgoal;
        }
        finalinital = false;
    }

    /**
     * @brief 处理目标点逻辑，动态发布 waypoint
     *
     * 功能：
     * - 根据机器人当前位置和中途目标队列判断当前目标点
     * - 判断是否到达当前目标点
     * - 发布 waypoint，确保中途目标按顺序发布
     * - 到达最终目标时，发布停止信号
     */
    void processGoals()
    {
        if (!midgoals.empty())
        {
            if (midgoals.size() > 1)
            {
                queueSort(midgoals);
            }
            current_goal = midgoals.front();
            if (current_goal.pose.position.x < pre_goal.pose.position.x)
            {
                ROS_INFO("接收到更近的目标点:X:%f,Y:%f,优先处理", current_goal.pose.position.x, current_goal.pose.position.y);
                midinital = true;
                waypoint_pub.publish(current_goal);
                midgoals.push(pre_goal);
                pre_goal = current_goal;
                midgoals.pop();
            }

            if (goal_reached || midinital)
            {
                current_goal = midgoals.front();
                if (current_goal == finalgoal && (midgoals.size() - 1 > 0))
                {
                    midgoals.pop();
                    current_goal = midgoals.front();
                }

                if (current_goal != pre_goal)
                {
                    midinital = false;
                    if (pre_goal.pose.position.x == -1)
                    {
                        ROS_INFO("发布第一个目标点: X:%f,Y:%f", current_goal.pose.position.x, current_goal.pose.position.y);
                        waypoint_pub.publish(current_goal);
                        midgoals.pop();
                        if (midgoals.empty())
                        {
                            if (!finalgoal_received)
                            {
                                ROS_INFO("加载最终发布点");
                            }
                            ROS_INFO("无中途发布点,队列补充最终发布点1");
                            midgoals.push(finalgoal);
                            midinital = true;
                        }
                        pre_goal = current_goal;
                    }
                    else if (abs(current_position - pre_goal.pose.position.x) < 0.05)
                    {
                        ROS_INFO("目标点: X:%f,Y:%f已到达,停留10s", pre_goal.pose.position.x, pre_goal.pose.position.y);
                        ros::Duration(10).sleep();
                        ROS_INFO("发布下一个目标点: X:%f,Y:%f", current_goal.pose.position.x, current_goal.pose.position.y);
                        waypoint_pub.publish(current_goal);
                        midgoals.pop();
                        if (midgoals.empty() && finalgoal_received)
                        {
                            ROS_INFO("无中途发布点,队列补充最终发布点2");
                            midgoals.push(finalgoal);
                            midinital = true;
                        }
                        pre_goal = current_goal;
                    }
                    else
                    {
                        float dis = abs(current_position - pre_goal.pose.position.x);
                        ROS_INFO("正在前往目标点: X:%f,Y:%f,距离为:%f", pre_goal.pose.position.x, pre_goal.pose.position.y, dis);
                    }
                }
            }
        }
    }

    /**
     * @brief 状态回调函数
     * @param msg 接收到的机器人状态 std_msgs::Bool 消息
     *
     * 功能：
     * - 根据消息更新 goal_reached 标志
     */
    void statusCallback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
        {
            goal_reached = true;
        }
        else
        {
            goal_reached = false;
        }
    }

private:
    ros::NodeHandle nh;                                 ///< ROS 节点句柄
    ros::Publisher waypoint_pub, auto_navigation_stop_pub; ///< 发布 waypoint 和自动导航停止信号
    ros::Subscriber midgoal_sub, finalgoal_sub, odom_sub; ///< 订阅中途目标、最终目标和 odom
    ros::Subscriber status_sub;                          ///< 订阅机器人状态
    std::queue<geometry_msgs::PoseStamped> midgoals;    ///< 中途目标队列
    geometry_msgs::PoseStamped current_goal;            ///< 当前目标点
    geometry_msgs::PoseStamped finalgoal;               ///< 最终目标点
    geometry_msgs::PoseStamped pre_goal;                ///< 上一个目标点

    double current_position, current_status;            ///< 当前机器人位置和速度
    bool finalgoal_received = false;                    ///< 是否接收到最终目标
    bool goal_reached = false;                          ///< 是否到达当前目标
    bool finalgoal_reached = false;                     ///< 是否到达最终目标
    bool midinital = true;                              ///< 是否处理中途目标初始化
    bool finalinital = true;                            ///< 是否最终目标初始化
};

/**
 * @brief 主程序入口
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 返回 0 表示正常退出
 *
 * 功能：
 * - 初始化 ROS 节点
 * - 创建 WaypointPublisher 对象
 * - 循环调用 processGoals，发布目标点
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_publisher");
    setlocale(LC_ALL, "");
    WaypointPublisher wp_publisher;
    ros::Rate loop_rate(10); // 10Hz
    while (ros::ok())
    {
        wp_publisher.processGoals();
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

/**
 * @file detectionPointPub.cpp
 * @brief 检测点分析与 midgoal 生成模块
 *
 * 该文件实现了一个 ROS 节点，用于接收检测点信息、机器人位姿和最终目标点，
 * 并根据检测点和机器人当前位置计算中间目标点 (midgoal)。
 * 功能：
 *  - 订阅 /cable/odom 获取机器人位姿
 *  - 订阅 /transformed_detection_info 获取检测点
 *  - 订阅 finalgoal 获取最终目标点
 *  - 根据检测点和机器人位姿生成 midgoal 点
 *  - 发布 midgoal 点供导航使用
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include "pipeline_robot/detection.h"
#include <vector>
#include <algorithm>

/**
 * @class DetectionAnalyzer
 * @brief 分析检测点并生成 midgoal 点
 *
 * 该类功能：
 *  - 缓存检测点信息，避免重复发布
 *  - 根据当前机器人位置和最终目标点选择合适的 midgoal 点
 *  - 发布 midgoal 点供导航模块使用
 */
class DetectionAnalyzer
{
public:
    /**
     * @brief 构造函数，初始化 ROS 订阅者与发布者
     *
     * 初始化以下订阅者：
     *  - /cable/odom：机器人位姿
     *  - /transformed_detection_info：检测点信息
     *  - finalgoal：最终目标点
     * 初始化发布者：
     *  - midgoal：中间目标点
     */
    DetectionAnalyzer();

    /**
     * @brief 更新机器人当前位置
     * @param msg nav_msgs::Odometry 消息，包含机器人当前位置
     *
     * 从 odom 消息中获取机器人 x 坐标，并更新 current_position_x
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    /**
     * @brief 更新最终目标点位置
     * @param msg geometry_msgs::PoseStamped 消息，包含最终目标点
     *
     * 从 finalgoal 消息中获取最终目标 x 坐标，并更新 final_position_x
     */
    void finalgoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    /**
     * @brief 接收并缓存检测点
     * @param msg pipeline_robot::detection 消息，包含检测点信息
     *
     * 功能：
     *  - 仅处理 x 坐标在 [0.5, 3] 范围内的检测点
     *  - 忽略超过最终目标点的检测点
     *  - 避免缓存重复的检测点
     */
    void detectionCallback(const pipeline_robot::detection::ConstPtr &msg);

    /**
     * @brief 从缓存检测点中选择最小 x 坐标生成 midgoal 点并发布
     *
     * 功能：
     *  - 检查新 midgoal 是否与已发布 midgoal 点重复
     *  - 发布 midgoal 到 "midgoal" 话题
     *  - 更新已发布 midgoal 缓存
     */
    void publishMidGoal();

private:
    ros::NodeHandle nh;                    ///< ROS 节点句柄
    ros::Subscriber odom_sub;             ///< 订阅 /cable/odom
    ros::Subscriber detection_sub;        ///< 订阅 /transformed_detection_info
    ros::Subscriber finalgoal_sub;        ///< 订阅 finalgoal
    ros::Publisher midgoal_pub;           ///< 发布 midgoal

    std::vector<float> detections;        ///< 缓存检测点 x 坐标
    std::vector<float> published_positions; ///< 已发布 midgoal x 坐标
    float current_position_x = 0;         ///< 当前机器人 x 坐标
    float final_position_x = 0.1;         ///< 最终目标 x 坐标

    /**
     * @brief 判断检测点是否已存在缓存中
     * @param new_detection_x 新检测点 x 坐标
     * @return true 如果缓存中存在相似点，否则 false
     *
     * 相似性判定：
     *  - 若 |new_detection_x - existing_x| < 1，则认为相似
     */
    bool isSimilar(float new_detection_x);

    /**
     * @brief 判断 midgoal 是否已发布
     * @param x midgoal x 坐标
     * @return true 如果已发布点中存在相似点，否则 false
     *
     * 相似性判定：
     *  - 若 |x - published_x| < 0.5，则认为相似
     */
    bool isSimilarPublished(float x);
};

// =================== 函数实现 ===================

DetectionAnalyzer::DetectionAnalyzer()
{
    odom_sub = nh.subscribe("/cable/odom", 10, &DetectionAnalyzer::odomCallback, this);
    detection_sub = nh.subscribe("/transformed_detection_info", 10, &DetectionAnalyzer::detectionCallback, this);
    finalgoal_sub = nh.subscribe("finalgoal", 10, &DetectionAnalyzer::finalgoalCallback, this);

    midgoal_pub = nh.advertise<geometry_msgs::PoseStamped>("midgoal", 10);
}

void DetectionAnalyzer::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_position_x = msg->pose.pose.position.x;
}

void DetectionAnalyzer::finalgoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    final_position_x = msg->pose.position.x;
}

void DetectionAnalyzer::detectionCallback(const pipeline_robot::detection::ConstPtr &msg)
{
    if (msg->point_stamped.point.x > 0.5 && msg->point_stamped.point.x < 3)
    {
        float new_detection_x = msg->point_stamped.point.x + current_position_x;
        if (new_detection_x > final_position_x)
            return;

        if (!isSimilar(new_detection_x))
            detections.push_back(new_detection_x);
    }
}

void DetectionAnalyzer::publishMidGoal()
{
    if (!detections.empty())
    {
        auto min_it = std::min_element(detections.begin(), detections.end());
        float min_detection_x = *min_it;

        if (!isSimilarPublished(min_detection_x))
        {
            geometry_msgs::PoseStamped midgoal;
            midgoal.header.stamp = ros::Time::now();
            midgoal.header.frame_id = "odom";
            midgoal.pose.position.x = min_detection_x - 0.5;
            midgoal.pose.orientation.w = 1.0;
            midgoal_pub.publish(midgoal);

            published_positions.push_back(min_detection_x);
        }
        detections.erase(min_it);
    }
}

bool DetectionAnalyzer::isSimilar(float new_detection_x)
{
    for (const float &x : detections)
    {
        if (std::fabs(x - new_detection_x) < 1)
            return true;
    }
    return false;
}

bool DetectionAnalyzer::isSimilarPublished(float x)
{
    for (const float &published_x : published_positions)
    {
        if (std::fabs(published_x - x) < 0.5)
            return true;
    }
    return false;
}

/**
 * @brief 主程序入口
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 返回 0 表示正常退出
 *
 * 功能：
 *  - 初始化 ROS 节点
 *  - 创建 DetectionAnalyzer 对象
 *  - 循环处理回调并周期性发布 midgoal
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection_analyzer");
    DetectionAnalyzer analyzer;

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        analyzer.publishMidGoal();
        loop_rate.sleep();
    }

    return 0;
}

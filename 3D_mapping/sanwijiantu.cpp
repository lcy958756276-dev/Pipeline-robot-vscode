/**
 * @file da_octree_node.cpp
 * @brief 通过订阅 Distance-Angle 数据和外部 odom，生成稀疏化点云并构建八叉树
 *
 * 功能：
 * - 订阅 `/distance_angle` 消息，将距离角度数据转换为点云。
 * - 订阅 `/cable/odom` 获取机器人位姿。
 * - 使用 pcl::octree::OctreePointCloudVoxelCentroid 对点云进行稀疏化。
 * - 发布整体稀疏化点云 `/overall_map_cloud`。
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pipeline_robot/DistanceAngle.h>

/**
 * @class DA_Octree_Node
 * @brief 通过 Distance-Angle 数据生成点云并构建八叉树的节点类
 */
class DA_Octree_Node {
public:
    /**
     * @brief 构造函数，初始化订阅者、发布者和八叉树
     */
    DA_Octree_Node() : octree_(0.000001f), odom_received_(false) {
        ros::NodeHandle nh;

        // 订阅 Distance-Angle
        da_sub_ = nh.subscribe("/distance_angle", 1000, &DA_Octree_Node::daCallback, this);

        // 订阅外部 odom
        odom_sub_ = nh.subscribe("/cable/odom", 10, &DA_Octree_Node::odomCallback, this);

        // 发布稀疏化点云
        cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/overall_map_cloud", 1);

        cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_->is_dense = false;
    }

    /**
     * @brief odom 回调函数，更新当前机器人位姿
     * @param msg 当前里程计消息（nav_msgs::Odometry）
     */
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pose_ = msg->pose.pose;
        odom_received_ = true;
    }

    /**
     * @brief Distance-Angle 回调函数，将距离角度数据转换为点并更新八叉树
     * @param msg Distance-Angle 消息（pipeline_robot::DistanceAngle）
     */
    void daCallback(const pipeline_robot::DistanceAngle::ConstPtr& msg) {
        if (!odom_received_) {
            ROS_WARN_THROTTLE(5, "Waiting for /cable/odom ...");
            return;
        }

        ros::Time current_time = ros::Time::now();
        float angle_rad = msg->angle * M_PI / 180.0;

        pcl::PointXYZ point;
        point.x = current_pose_.position.x;            ///< 使用外部 odom 的 X
        point.y = msg->distance * sin(angle_rad) * 0.01;
        point.z = msg->distance * cos(angle_rad) * (-0.01);

        cloud_->points.push_back(point);

        // 更新八叉树
        octree_.setInputCloud(cloud_);
        octree_.addPointsFromInputCloud();

        pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
        octree_.getOccupiedVoxelCenters(outputCloud->points);

        sensor_msgs::PointCloud2 outputMsg;
        pcl::toROSMsg(*outputCloud, outputMsg);
        outputMsg.header.stamp = current_time;
        outputMsg.header.frame_id = "odom";  ///< 与外部 odom 保持一致
        cloud_pub_.publish(outputMsg);

        ROS_INFO_STREAM("Octree voxels: " << octree_.getLeafCount()
                        << " | Input points: " << cloud_->size());
    }

private:
    ros::Subscriber da_sub_;        ///< Distance-Angle 数据订阅者
    ros::Subscriber odom_sub_;      ///< 外部 odom 数据订阅者
    ros::Publisher cloud_pub_;      ///< 发布稀疏化点云

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_; ///< 原始点云存储
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree_; ///< 八叉树对象

    geometry_msgs::Pose current_pose_; ///< 当前机器人位姿
    bool odom_received_;               ///< odom 是否已接收标志
};

/**
 * @brief 主函数，初始化 ROS 节点并启动 DA_Octree_Node
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出状态
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "da_octree_node");
    DA_Octree_Node node;
    ros::spin();
    return 0;
}

/**
 * @file octree_map_builder.cpp
 * @brief 使用八叉树对累计点云进行存储与管理，同时根据机器人位姿进行点云变换和统计。
 * 
 * 功能：
 * - 接收机器人里程计信息，记录当前位置。
 * - 接收点云信息，并根据里程计变换至 odom 坐标系。
 * - 使用 pcl::octree::OctreePointCloudVoxelCentroid 构建八叉树。
 * - 累计统计点云数量和八叉树体素数量。
 * - 发布整体地图点云。
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <Eigen/Dense>

/** @brief 八叉树分辨率 */
float resolution = 0.01f;

/** @brief 八叉树对象，用于存储体素中心点 */
pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree(resolution);

/** @brief TF 监听器指针，用于坐标变换 */
tf::TransformListener* tfListener;

// 统计相关变量
size_t cumulativePoints = 0;        ///< 累计接收点数
size_t currentVoxels = 0;           ///< 当前体素数量
Eigen::Vector3d lastPosition = Eigen::Vector3d::Zero();  ///< 上一时刻机器人位置
Eigen::Vector3d currentPosition = Eigen::Vector3d::Zero(); ///< 当前机器人位置
bool positionInitialized = false;   ///< 位置是否初始化标志

/**
 * @brief odometry 回调函数，更新机器人当前位置
 * 
 * @param msg 当前机器人里程计信息（nav_msgs::Odometry）
 * @return void
 */
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    currentPosition = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);

    if(!positionInitialized) {
        lastPosition = currentPosition;
        positionInitialized = true;
    }
}

/**
 * @brief 点云回调函数，将接收到的点云变换到 odom 坐标系并更新八叉树
 * 
 * @param cloudMsg 接收到的点云（sensor_msgs::PointCloud2）
 * @return void
 */
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloudMsg, *cloud);

    double distanceMoved = (currentPosition - lastPosition).norm();
    
    if(distanceMoved > 0.01)
    {
        lastPosition = currentPosition;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        if(tfListener->waitForTransform("odom", cloudMsg->header.frame_id, 
                                      cloudMsg->header.stamp, ros::Duration(1.0)))
        {
            pcl_ros::transformPointCloud("odom", *cloud, *transformedCloud, *tfListener);
            
            // 更新统计信息
            cumulativePoints += transformedCloud->size();
            
            // 更新八叉树
            octree.setInputCloud(transformedCloud);
            octree.addPointsFromInputCloud();
        }
    }
}

/**
 * @brief 主函数，初始化 ROS 节点、订阅者、发布者并循环处理点云
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出状态
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "octree_map_builder");
    ros::NodeHandle nh;
    
    tfListener = new tf::TransformListener();

    ros::Subscriber sub = nh.subscribe("/point_cloud", 10, cloudCallback);
    ros::Subscriber odomSub = nh.subscribe("/cable/odom", 10, odomCallback);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/overall_map_cloud", 1);

    ros::Rate rate(10);
    
    while(ros::ok())
    {
        ros::spinOnce();
        
        // 获取当前八叉树状态
        currentVoxels = octree.getLeafCount();
        
        // 计算内存占用（假设每个点占12字节：3个float）
        double rawMemoryMB = (cumulativePoints * sizeof(pcl::PointXYZ)) / (1024.0 * 1024.0);
        double octreeMemoryMB = (currentVoxels * sizeof(pcl::PointXYZ)) / (1024.0 * 1024.0);
        
        // 发布点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>);
        octree.getOccupiedVoxelCenters(outputCloud->points);
        
        sensor_msgs::PointCloud2 outputMsg;
        pcl::toROSMsg(*outputCloud, outputMsg);
        outputMsg.header.stamp = ros::Time::now();
        outputMsg.header.frame_id = "odom";
        pub.publish(outputMsg);
        
        // 输出统计信息
        // ROS_INFO_STREAM("Accumulated Points: " << cumulativePoints 
        //                << " (" << rawMemoryMB << " MB) | "
        //                << "Octree Voxels: " << currentVoxels 
        //                << " (" << octreeMemoryMB << " MB)");
        
        rate.sleep();
    }
    
    delete tfListener;
    return 0;
}

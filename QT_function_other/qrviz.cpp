/**
 * @file qrviz.cpp
 * @brief RViz 可视化封装类，用于在 Qt 界面中显示机器人、激光、地图和点云
 *
 * 功能：
 * - 在 Qt 的 QVBoxLayout 中嵌入 RViz 渲染面板
 * - 初始化 RViz VisualizationManager
 * - 添加基础显示层：网格、地图、激光扫描、机器人模型、点云
 * - 设置 Fixed Frame 为 odom
 */

#include "qrviz.h"

/**
 * @brief 构造函数，初始化 RViz 渲染面板及显示层
 * @param layout 父布局，用于嵌入渲染面板
 */
Qrviz(QVBoxLayout *layout)
{
    // 创建渲染面板并加入父布局
    render_panel_ = new rviz::RenderPanel();
    layout->addWidget(render_panel_);

    // 初始化 VisualizationManager
    manager_ = new rviz::VisualizationManager(render_panel_);
    ROS_ASSERT(manager_ != NULL);
    render_panel_->initialize(manager_->getSceneManager(), manager_);
    manager_->initialize();          // 初始化 manager
    manager_->removeAllDisplays();   // 删除已有所有图层
    manager_->startUpdate();         // 启动图层更新
    manager_->setFixedFrame("odom"); // 设置 Fixed Frame

    // 添加网格显示
    rviz::Display *grid_ = manager_->createDisplay("rviz/Grid", "adjustable grid", true);
    ROS_ASSERT(grid_ != NULL);
    grid_->subProp("Line Style")->setValue("Billboards");
    grid_->subProp("Color")->setValue(QColor(125, 125, 125));

    // 添加地图显示
    rviz::Display *map_ = manager_->createDisplay("rviz/Map", "adjustable map", true);
    map_->subProp("Topic")->setValue("/map");
    map_->subProp("Alpha")->setValue("0.7");
    map_->subProp("Color Scheme")->setValue("map");

    // 添加激光扫描显示
    rviz::Display *laser_ = manager_->createDisplay("rviz/LaserScan", "QLaser", true);
    laser_->subProp("Topic")->setValue("/scan");

    // 添加机器人模型显示
    rviz::Display *rot = manager_->createDisplay("rviz/RobotModel", "cobot", true);
    rot->subProp("Visual Enabled")->setValue("true");
    rot->subProp("Collision Enabled")->setValue("false");
    rot->subProp("Robot Description")->setValue("robot_description");
    rot->subProp("Alpha")->setValue("1");

    // 添加点云显示
    rviz::Display *point_cloud_ = manager_->createDisplay("rviz/PointCloud2", "PointCloud2", true);
    ROS_ASSERT(point_cloud_ != NULL);
    point_cloud_->subProp("Topic")->setValue("/overall_map_cloud");
    point_cloud_->subProp("Color Transformer")->setValue("Intensity");
    point_cloud_->subProp("Size (Pixels)")->setValue(3);

    // 启动显示更新
    manager_->startUpdate();
}

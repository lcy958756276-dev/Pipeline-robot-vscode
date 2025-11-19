#!/usr/bin/env python3
# coding: UTF-8
"""
@file point_transformer.py
@brief 检测点坐标转换模块

该文件实现了一个 ROS 节点，用于将来自相机坐标系的检测点信息
(detection_new 消息) 转换到全局 odom 坐标系下，并发布转换后的检测点。

功能：
- 订阅 /_detection_info 接收检测点信息
- 使用 tf2 将检测点从相机坐标系转换到 odom 坐标系
- 发布转换后的检测点到 /transformed_detection_info
- 支持检测点的最小值和最大值坐标转换
"""

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs
from pipeline_robot.msg import detection_new

class PointTransformer:
    """
    @class PointTransformer
    @brief 检测点坐标转换类

    该类功能：
    - 初始化 ROS 节点和 TF2 监听器
    - 接收来自 /_detection_info 的检测点消息
    - 将检测点坐标从原始坐标系转换到 odom 坐标系
    - 发布转换后的检测点消息到 /transformed_detection_info
    """

    def __init__(self):
        """
        @brief 构造函数，初始化 ROS 节点、TF2 和订阅/发布者

        功能：
        - 初始化 ROS 节点 'point_transformer'
        - 初始化 tf2 缓冲区和监听器
        - 订阅 /_detection_info 接收 detection_new 消息
        - 创建 /transformed_detection_info 发布者
        """
        rospy.init_node('point_transformer', anonymous=True)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.Subscriber("/_detection_info", detection_new, self.detection_callback)
        self.detection_pub = rospy.Publisher("/transformed_detection_info", detection_new, queue_size=10)

    def detection_callback(self, detection_data):
        """
        @brief 检测点回调函数，将检测点从原始坐标系转换到 odom 坐标系并发布

        @param detection_data detection_new
            输入的检测点消息，包含以下字段：
            - class_name: 检测类别名称
            - score: 检测置信度
            - point_stamped: 检测点坐标
            - min_stamped: 检测点最小值坐标
            - max_stamped: 检测点最大值坐标

        @return None
            函数无返回值

        功能：
        - 使用 tf2 查找检测点坐标到 odom 的变换
        - 对 point_stamped、min_stamped 和 max_stamped 进行坐标变换
        - 将转换后的检测点封装到新的 detection_new 消息并发布
        - 如果发生 TF2 错误，输出错误日志
        """
        try:
            transform = self.tf_buffer.lookup_transform('odom',
                                                        detection_data.point_stamped.header.frame_id,
                                                        rospy.Time(0),
                                                        rospy.Duration(1.0))
            
            transformed_point = tf2_geometry_msgs.do_transform_point(detection_data.point_stamped, transform)
            transformed_point_min = tf2_geometry_msgs.do_transform_point(detection_data.min_stamped, transform)
            transformed_point_max = tf2_geometry_msgs.do_transform_point(detection_data.max_stamped, transform)
            
            rospy.loginfo("Transformed point in camera_link: (%s,%.2f, %.2f, %.2f)", 
                          detection_data.class_name,
                          transformed_point.point.x, 
                          transformed_point.point.y, 
                          transformed_point.point.z)
            
            new_detection = detection_new()
            new_detection.class_name = detection_data.class_name
            new_detection.score = detection_data.score
            # new_detection.image_data = detection_data.image_data  
            new_detection.min_stamped = transformed_point_min 
            new_detection.max_stamped = transformed_point_max

            self.detection_pub.publish(new_detection)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF2 Error: %s", str(e))

if __name__ == '__main__':
    """
    @brief 主程序入口，创建 PointTransformer 对象并进入 ROS 循环
    """
    pt = PointTransformer()
    rospy.spin()

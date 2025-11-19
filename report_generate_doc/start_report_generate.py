#!/usr/bin/env python
"""
作用：
    该模块用于监听 ROS 话题 `whether_report_generate`，根据接收到的
    Bool 值来启动或终止 auto_report.launch，从而控制报告生成流程。

主要功能：
    - 订阅 whether_report_generate 话题
    - 在收到 True 时启动 auto_report.launch
    - 在收到 False 时终止 auto_report.launch

使用的库：
    - rospy：ROS Python 客户端库
    - std_msgs.msg：标准 ROS 消息类型
    - subprocess：用于启动外部 roslaunch 进程
"""

import rospy
from std_msgs.msg import Bool
import subprocess

roslaunch_process = None


def handle_roslaunch(is_start: bool) -> None:
    """
    作用：
        根据 is_start 的值启动或终止 auto_report.launch 进程。

    参数：
        is_start (bool):
            - True：启动报告生成流程
            - False：终止已启动的流程

    返回：
        None
    """
    global roslaunch_process

    if is_start:
        if roslaunch_process is None:
            try:
                rospy.loginfo("===================================subprocess start========================")
                roslaunch_process = subprocess.Popen(
                    ['roslaunch', 'pipeline_robot', 'auto_report.launch']
                )
                rospy.loginfo("report_generate started.")
            except subprocess.CalledProcessError as e:
                rospy.logerr("Failed to start report_generate: %s", e)
        else:
            rospy.loginfo("report_generate is already running.")
    else:
        if roslaunch_process is not None:
            roslaunch_process.terminate()
            roslaunch_process = None
            rospy.loginfo("report_generate terminated.")
        else:
            rospy.loginfo("No report_generate process to terminate.")


def report_generate_callback(msg: Bool) -> None:
    """
    作用：
        ROS Subscriber 的回调函数，用于接收是否需要生成报告的指令。

    参数：
        msg (Bool):
            从 whether_report_generate 话题收到的布尔值。

    返回：
        None
    """
    rospy.loginfo("Received report_generate: %s", msg.data)
    handle_roslaunch(msg.data)


def listener() -> None:
    """
    作用：
        初始化 ROS 节点并开始监听 whether_report_generate 话题。

    参数：
        无

    返回：
        None
    """
    rospy.init_node('report_generate_listener', anonymous=True)
    rospy.Subscriber("whether_report_generate", Bool, report_generate_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

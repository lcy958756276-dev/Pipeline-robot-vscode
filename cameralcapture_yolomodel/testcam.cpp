namespace cameralcapture {
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include "pipeline_robot/compressedRGBD.h" // 包含自定义消息

/**
 * @brief Main entry point for RTSP reader node
 * 
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int 0 if success, -1 if failure
 */



int main(int argc, char** argv) {
    ros::init(argc, argv, "rtsp_reader_node"); // 初始化 ROS 节点
    ros::NodeHandle nh;

    // 发布自定义 RGBD 压缩消息
    ros::Publisher rgbd_pub = nh.advertise<pipeline_robot::compressedRGBD>("/camera/rgb_depth/compressed", 1);

    std::string rtsp_url = "rtsp://admin:123456@192.168.8.10/Streaming/Channels/2"; // RTSP 视频流 URL
    cv::VideoCapture cap(rtsp_url);

    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open RTSP stream: %s", rtsp_url.c_str());
        return -1; // 如果无法打开 RTSP 流，返回错误
    } else {
        ROS_INFO("Successfully connected to RTSP stream: %s", rtsp_url.c_str());
    }

    ros::Rate loop_rate(30); // 设定循环频率为 30 FPS
    int frame_count = 0; // 帧计数器

    while (ros::ok()) {
        cv::Mat frame;
        if (!cap.read(frame)) {
            ROS_WARN("Failed to read frame from RTSP stream, retrying...");
            cap.release(); // 释放当前的流
            ros::Duration(1.0).sleep(); // 等待 1 秒后重新连接
            cap.open(rtsp_url); // 重新打开 RTSP 流
            if (cap.isOpened()) {
                ROS_INFO("Reconnected to RTSP stream successfully.");
            } else {
                ROS_ERROR("Reconnect failed. Will retry...");
            }
            continue; // 如果读取失败，则继续循环
        }

        frame_count++; // 增加帧计数
        //ROS_INFO("Received frame #%d, size: %dx%d", frame_count, frame.cols, frame.rows);

        // --- 压缩 RGB 图像 ---
        std::vector<uchar> buf; // 存储压缩后的图像数据
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 75}; // 压缩质量设置为 75
        cv::imencode(".jpg", frame, buf, params); // 压缩图像为 JPEG 格式

        // 构建自定义 RGBD 消息
        pipeline_robot::compressedRGBD msg;
        msg.rgb.header.stamp = ros::Time::now(); // 当前时间戳
        msg.rgb.header.frame_id = "camera"; // 设置帧 ID
        msg.rgb.format = "jpeg"; // 图像格式
        msg.rgb.data.assign(buf.begin(), buf.end()); // 将压缩图像数据赋值给消息

        // 如果 depth 数据有，也可以在这里填充 msg.depth
        // msg.depth = ...

        rgbd_pub.publish(msg); // 发布消息

        ros::spinOnce(); // 处理回调函数
        loop_rate.sleep(); // 按照设定的频率睡眠
    }

    cap.release(); // 释放视频流
    return 0; // 正常结束
}
} // namespace cameralcapture
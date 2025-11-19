/**
 * @file sensor_bridge_node.cpp
 * @brief TCP 数据接收与 Nano/Orin 温度监测 ROS 节点
 *
 * 功能：
 * - 从 TCP 服务器接收传感器数据（温度/压力）并解析发布
 * - 发布 NVIDIA Orin/Nano CPU 和 GPU 温度
 * - TCP 循环自动接收并处理不完整行
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <string>
#include <sstream>
#include <fstream>

// ------------------ 全局发布器 ------------------
ros::Publisher pub_temp;        /**< 发布解析的温度 /temperature_data */
ros::Publisher pub_press;       /**< 发布解析的压力 /pressVal */
ros::Publisher pub_cpu_temp;    /**< 发布 Nano/Orin CPU 温度 /orin_cpu_temp */
ros::Publisher pub_gpu_temp;    /**< 发布 Nano/Orin GPU 温度 /orin_gpu_temp */

/**
 * @brief 读取系统文件中存储的 CPU/GPU 温度
 * @param path 系统 thermal_zone 文件路径
 * @return 温度值（摄氏度），失败返回 -1.0
 */
float readThermal(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) return -1.0;
    int milliC;
    file >> milliC;
    return milliC / 1000.0;
}

/**
 * @brief 解析 TCP 接收到的一行传感器数据并发布 ROS 消息
 * 
 * 数据格式示例：
 * - "NTC:72.5" -> 温度传感器值
 * - "AP:101.3" -> 压力传感器值
 * 
 * @param line 原始数据行
 */
void parseAndPublish(const std::string& line) {
    float ntc_val = -1, ap_val = -1;
    size_t pos;

    pos = line.find("NTC:");
    if (pos != std::string::npos) {
        ntc_val = std::stof(line.substr(pos + 4));
    }

    pos = line.find("AP:");
    if (pos != std::string::npos) {
        ap_val = std::stof(line.substr(pos + 3));
    }

    if (ntc_val >= 0) {
        std_msgs::Float32 msg;
        msg.data = (ntc_val-32)*5/9;  // 华氏转摄氏
        pub_temp.publish(msg);
        ROS_INFO("Published /temperature_data = %.2f", ntc_val);
    }

    if (ap_val >= 0) {
        std_msgs::Float32 msg;
        msg.data = ap_val;
        pub_press.publish(msg);
        ROS_INFO("Published /pressVal = %.2f", ap_val);
    }
}

/**
 * @brief TCP 接收循环
 * 
 * 从指定 IP:端口接收数据，每次读取后按行解析
 * 并调用 parseAndPublish 处理。处理不完整行缓存。
 * 
 * @param ip TCP 服务器 IP
 * @param port TCP 服务器端口
 */
void tcpLoop(const std::string& ip, int port) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        ROS_ERROR("Socket creation failed");
        return;
    }

    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr);

    if (connect(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        ROS_ERROR("Connection failed to %s:%d", ip.c_str(), port);
        return;
    }

    ROS_INFO("Connected to server %s:%d", ip.c_str(), port);

    char buffer[1024];
    std::string leftover;

    while (ros::ok()) {
        ssize_t n = read(sockfd, buffer, sizeof(buffer) - 1);
        if (n <= 0) {
            ROS_WARN("Server disconnected");
            break;
        }
        buffer[n] = '\0';
        std::string data = leftover + buffer;

        std::istringstream iss(data);
        std::string line;
        while (std::getline(iss, line)) {
            if (!line.empty()) parseAndPublish(line);
        }

        // 处理不完整的一行
        if (data.back() != '\n')
            leftover = data.substr(data.find_last_of('\n') + 1);
        else
            leftover.clear();

        ros::spinOnce();
    }

    close(sockfd);
}

/**
 * @brief 定时读取 Orin/Nano CPU 和 GPU 温度并发布 ROS 消息
 */
void publishNanoTemp() {
    float cpu_temp = readThermal("/sys/class/thermal/thermal_zone0/temp");
    float gpu_temp = readThermal("/sys/class/thermal/thermal_zone1/temp");

    std_msgs::Float32 msg_cpu, msg_gpu;
    msg_cpu.data = cpu_temp;
    msg_gpu.data = gpu_temp;

    pub_cpu_temp.publish(msg_cpu);
    pub_gpu_temp.publish(msg_gpu);

    ROS_INFO("Orin CPU Temp: %.2f °C, GPU Temp: %.2f °C", cpu_temp, gpu_temp);
}

/**
 * @brief 主函数
 * 
 * 初始化 ROS 节点，配置参数，创建发布器和定时器
 * 并调用 tcpLoop 开始接收数据
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_bridge_node");
    ros::NodeHandle nh;

    std::string ip;
    int port;
    nh.param<std::string>("ip", ip, "192.168.8.11");
    nh.param<int>("port", port, 12345);

    // 发布话题
    pub_temp = nh.advertise<std_msgs::Float32>("/temperature_data", 10);
    pub_press = nh.advertise<std_msgs::Float32>("/pressVal", 10);
    pub_cpu_temp = nh.advertise<std_msgs::Float32>("/orin_cpu_temp", 10);
    pub_gpu_temp = nh.advertise<std_msgs::Float32>("/orin_gpu_temp", 10);

    // CPU/GPU 温度定时器，每秒发布一次
    ros::Timer temp_timer = nh.createTimer(ros::Duration(1.0), [](const ros::TimerEvent&){
        publishNanoTemp();
    });

    // TCP 循环（阻塞式）
    tcpLoop(ip, port);

    ros::spin();
    return 0;
}

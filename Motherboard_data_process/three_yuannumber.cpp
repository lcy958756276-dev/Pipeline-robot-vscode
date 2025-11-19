/**
 * @file tcp_rph_extractor.cpp
 * @brief TCP 接收 Roll-Pitch-Heading (RPH) 数据并发布 ROS 话题
 *
 * 功能：
 * - 从 TCP 服务器接收 RPH 数据
 * - 解析 RPH 字符串（R=, P=, H=）
 * - 分别发布到 /roll、/pitch、/heading 话题
 * - 支持 TCP 自动重连，并发布连接状态到 /connection_status
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>  // 用于发布单个浮点数
#include <std_msgs/String.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sstream>
#include <mutex>

// ------------------ 全局变量（线程安全保护） ------------------
std::mutex g_conn_mutex;          /**< TCP 连接互斥锁 */
int g_sockfd = -1;                /**< TCP 套接字文件描述符 */
std::string g_ip;                 /**< TCP 服务器 IP */
int g_port;                        /**< TCP 服务器端口 */
ros::Publisher g_status_pub;       /**< 发布连接状态消息 */

// ------------------ 函数声明 ------------------

/**
 * @brief 连接 TCP 服务器
 * 
 * @param ip 服务器 IP 地址
 * @param port 服务器端口
 * @return true 连接成功
 * @return false 连接失败
 */
bool connectToServer(const std::string& ip, int port);

/**
 * @brief 发布 TCP 连接状态到 /connection_status
 * 
 * @param status 状态字符串，如 "CONNECTED" 或 "DISCONNECTED"
 */
void publishStatus(const std::string& status);

/**
 * @brief 解析 RPH 数据并发布到对应 ROS 话题
 * 
 * 数据格式示例：
 * - "R=1.23" -> Roll
 * - "P=2.34" -> Pitch
 * - "H=3.45" -> Heading
 * 
 * @param data 原始数据字符串
 * @param roll_pub 发布 Roll 的 ROS Publisher
 * @param pitch_pub 发布 Pitch 的 ROS Publisher
 * @param heading_pub 发布 Heading 的 ROS Publisher
 */
void parseAndPublishData(const std::string& data, 
                        ros::Publisher& roll_pub,
                        ros::Publisher& pitch_pub,
                        ros::Publisher& heading_pub);

/**
 * @brief TCP 数据接收循环
 * 
 * 持续读取 TCP 数据，并按行解析 RPH，发布原始数据和解析数据
 * 
 * @param raw_pub 发布原始数据的 ROS Publisher
 * @param roll_pub 发布 Roll 的 ROS Publisher
 * @param pitch_pub 发布 Pitch 的 ROS Publisher
 * @param heading_pub 发布 Heading 的 ROS Publisher
 */
void tcpDataLoop(ros::Publisher& raw_pub, 
                ros::Publisher& roll_pub,
                ros::Publisher& pitch_pub,
                ros::Publisher& heading_pub);

/**
 * @brief 处理 TCP 断开连接
 * 
 * 支持自动重连，达到最大重试次数则关闭 ROS 节点
 * 
 * @param max_reconnect 最大重连次数
 * @param interval 重连间隔（秒）
 */
void handleDisconnection(int max_reconnect, float interval);

// ------------------ main ------------------
int main(int argc, char** argv) {
    ros::init(argc, argv, "tcp_rph_extractor");
    ros::NodeHandle nh("~");

    // 参数配置
    nh.param<std::string>("ip", g_ip, "192.168.8.11");
    nh.param<int>("port", g_port, 12345);
    int max_reconnect;
    float reconnect_interval;
    nh.param<int>("max_reconnect", max_reconnect, 5);
    nh.param<float>("reconnect_interval", reconnect_interval, 3.0);

    // 初始化ROS话题
    ros::Publisher raw_pub = nh.advertise<std_msgs::String>("/raw_data", 100);
    ros::Publisher roll_pub = nh.advertise<std_msgs::Float32>("/roll", 100);
    ros::Publisher pitch_pub = nh.advertise<std_msgs::Float32>("/pitch", 100);
    ros::Publisher heading_pub = nh.advertise<std_msgs::Float32>("/heading", 100);
    g_status_pub = nh.advertise<std_msgs::String>("/connection_status", 10);

    // 设置调试级别
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_INFO("Starting TCP RPH extractor");

    // 主循环
    while (ros::ok()) {
        if (connectToServer(g_ip, g_port)) {
            tcpDataLoop(raw_pub, roll_pub, pitch_pub, heading_pub);
        } else {
            handleDisconnection(max_reconnect, reconnect_interval);
        }
        ros::spinOnce();
    }
    close(g_sockfd);
    return 0;
}

// ------------------ 函数实现 ------------------

bool connectToServer(const std::string& ip, int port) {
    std::lock_guard<std::mutex> lock(g_conn_mutex);
    ROS_INFO_STREAM("Connecting to " << ip << ":" << port);

    g_sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_sockfd < 0) {
        ROS_ERROR_STREAM("Socket error: " << strerror(errno));
        return false;
    }

    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr) <= 0) {
        ROS_ERROR("Invalid IP format");
        return false;
    }

    if (connect(g_sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        ROS_ERROR_STREAM("Connection failed: " << strerror(errno));
        return false;
    }

    ROS_INFO("Connected to server");
    publishStatus("CONNECTED");
    return true;
}

void publishStatus(const std::string& status) {
    std_msgs::String msg;
    msg.data = "[STATUS] " + status + " | IP: " + g_ip;
    g_status_pub.publish(msg);
    ROS_DEBUG_STREAM("Status published: " << msg.data);
}

void tcpDataLoop(ros::Publisher& raw_pub, 
                ros::Publisher& roll_pub,
                ros::Publisher& pitch_pub,
                ros::Publisher& heading_pub) {
    char buffer[4096];
    std::string remaining_data;

    while (ros::ok()) {
        ssize_t valread = read(g_sockfd, buffer, sizeof(buffer) - 1);
        if (valread <= 0) {
            ROS_WARN_STREAM("Read error: " << (valread == 0 ? "EOF" : strerror(errno)));
            break;
        }

        buffer[valread] = '\0';
        std::string raw_data = remaining_data + buffer;
        size_t last_newline = raw_data.find_last_of('\n');

        if (last_newline != std::string::npos) {
            // 发布原始数据
            std_msgs::String raw_msg;
            raw_msg.data = "[RAW] " + raw_data.substr(0, 100);  // 截断前100字符
            raw_pub.publish(raw_msg);

            // 解析RPH数据
            parseAndPublishData(raw_data.substr(0, last_newline), roll_pub, pitch_pub, heading_pub);
            remaining_data = raw_data.substr(last_newline + 1);
        } else {
            remaining_data = raw_data;
            ROS_DEBUG("Buffering incomplete packet");
        }
        memset(buffer, 0, sizeof(buffer));
    }
}

void parseAndPublishData(const std::string& data, 
                        ros::Publisher& roll_pub,
                        ros::Publisher& pitch_pub,
                        ros::Publisher& heading_pub) {
    std::istringstream iss(data);
    std::string line;
    float roll = 0, pitch = 0, heading = 0;
    bool has_rph = false;

    while (std::getline(iss, line)) {
        if (line.empty()) continue;
        ROS_DEBUG_STREAM("Processing: " << line);

        try {
            if (line.find("R=") == 0) {
                roll = std::stof(line.substr(2));
                has_rph = true;
            } 
            else if (line.find("P=") == 0) {
                pitch = std::stof(line.substr(2));
                has_rph = true;
            } 
            else if (line.find("H=") == 0) {
                heading = std::stof(line.substr(2));
                if (has_rph) {
                    std_msgs::Float32 roll_msg, pitch_msg, heading_msg;
                    roll_msg.data = roll;
                    pitch_msg.data = pitch;
                    heading_msg.data = heading;
                    
                    roll_pub.publish(roll_msg);
                    pitch_pub.publish(pitch_msg);
                    heading_pub.publish(heading_msg);
                    
                    ROS_DEBUG_STREAM("Published RPH: R=" << roll 
                                    << ", P=" << pitch
                                    << ", H=" << heading);
                    has_rph = false;
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Parse error: " << e.what() << " in line: " << line);
        }
    }
}

void handleDisconnection(int max_reconnect, float interval) {
    static int attempts = 0;
    std::lock_guard<std::mutex> lock(g_conn_mutex);

    close(g_sockfd);
    publishStatus("DISCONNECTED");

    if (++attempts <= max_reconnect) {
        ROS_WARN_STREAM("Reconnecting (" << attempts << "/" << max_reconnect << ")");
        ros::Duration(interval).sleep();
    } else {
        ROS_FATAL("Max reconnection attempts reached");
        ros::shutdown();
    }
}

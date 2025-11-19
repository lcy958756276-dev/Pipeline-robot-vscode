/**
 * @file tcp_data_extractor.cpp
 * @brief TCP 客户端 ROS 节点，用于接收传感器数据并解析为 XYZ 坐标点和 Distance-Angle 消息。
 *
 * 功能包括：
 * - 建立 TCP 连接，接收原始数据。
 * - 解析 XYZ 坐标并发布到 /parsed_points。
 * - 解析 Distance-Angle 数据并发布到 /distance_angle。
 * - 发布原始数据到 /raw_data。
 * - 发布连接状态到 /connection_status。
 * - 自动处理 TCP 断开和重连。
 */

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <pipeline_robot/DistanceAngle.h>  // 自定义消息类型
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sstream>
#include <mutex>

/** @brief 全局变量，线程安全保护 TCP 连接 */
std::mutex g_conn_mutex;
int g_sockfd = -1;                     /**< TCP 套接字 */
std::string g_ip;                       /**< TCP 服务器 IP */
int g_port;                              /**< TCP 服务器端口 */
ros::Publisher g_status_pub;            /**< ROS 连接状态发布器 */

/** @brief 尝试连接到 TCP 服务器
 *  @param ip TCP 服务器 IP 地址
 *  @param port TCP 服务器端口号
 *  @return 成功连接返回 true，失败返回 false
 */
bool connectToServer(const std::string& ip, int port);

/** @brief 发布当前连接状态到 /connection_status
 *  @param status 状态字符串，例如 "CONNECTED" 或 "DISCONNECTED"
 */
void publishStatus(const std::string& status);

/** @brief 解析原始数据并发布坐标点和距离角度信息
 *  @param data 待解析的原始数据字符串
 *  @param point_pub 用于发布 geometry_msgs::PointStamped 的 ROS Publisher
 *  @param da_pub 用于发布 pipeline_robot::DistanceAngle 的 ROS Publisher
 */
void parseAndPublishData(const std::string& data, 
                        ros::Publisher& point_pub,
                        ros::Publisher& da_pub);

/** @brief 处理 TCP 断开连接，并根据最大重连次数尝试重新连接
 *  @param max_reconnect 最大重连次数
 *  @param interval 重连间隔时间（秒）
 */
void handleDisconnection(int max_reconnect, float interval);

/** @brief TCP 数据接收循环
 *  从 TCP 套接字接收数据，处理缓存，发布原始数据，同时调用解析函数
 *  @param raw_pub 用于发布 std_msgs::String 的 ROS Publisher（原始数据）
 *  @param point_pub 用于发布 geometry_msgs::PointStamped 的 ROS Publisher
 *  @param da_pub 用于发布 pipeline_robot::DistanceAngle 的 ROS Publisher
 */
void tcpDataLoop(ros::Publisher& raw_pub, 
                ros::Publisher& point_pub,
                ros::Publisher& da_pub);

/**
 * @brief 主函数
 *  初始化 ROS 节点和参数，创建话题发布器，启动 TCP 数据接收循环
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "tcp_data_extractor");
    ros::NodeHandle nh("~");

    // 参数配置
    nh.param<std::string>("ip", g_ip, "192.168.8.11");
    nh.param<int>("port", g_port, 12345);
    int max_reconnect;
    float reconnect_interval;
    nh.param<int>("max_reconnect", max_reconnect, 5);
    nh.param<float>("reconnect_interval", reconnect_interval, 3.0);

    // 初始化 ROS 话题
    ros::Publisher raw_pub = nh.advertise<std_msgs::String>("/raw_data", 100);
    ros::Publisher point_pub = nh.advertise<geometry_msgs::PointStamped>("/parsed_points", 100);
    ros::Publisher da_pub = nh.advertise<pipeline_robot::DistanceAngle>("/distance_angle", 100);
    g_status_pub = nh.advertise<std_msgs::String>("/connection_status", 10);

    // 设置调试级别
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_INFO("Starting TCP data extractor with XYZ and DA parsing");

    // 主循环
    while (ros::ok()) {
        if (connectToServer(g_ip, g_port)) {
            tcpDataLoop(raw_pub, point_pub, da_pub);
        } else {
            handleDisconnection(max_reconnect, reconnect_interval);
        }
        ros::spinOnce();
    }
    close(g_sockfd);
    return 0;
}

/**
 * @brief 连接 TCP 服务器
 *  创建套接字并尝试连接指定 IP 和端口
 *  @param ip TCP 服务器 IP 地址
 *  @param port TCP 服务器端口
 *  @return 成功返回 true，失败返回 false
 */
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

/**
 * @brief 发布连接状态消息
 *  通过 g_status_pub 发布状态，例如 CONNECTED 或 DISCONNECTED
 *  @param status 状态字符串
 */
void publishStatus(const std::string& status) {
    std_msgs::String msg;
    msg.data = "[STATUS] " + status + " | IP: " + g_ip;
    g_status_pub.publish(msg);
    ROS_DEBUG_STREAM("Status published: " << msg.data);
}

/**
 * @brief TCP 数据接收循环
 *  从 TCP 套接字接收数据，处理缓存，发布原始数据，同时调用解析函数
 *  @param raw_pub 发布原始字符串数据
 *  @param point_pub 发布解析后的 XYZ 坐标点
 *  @param da_pub 发布解析后的 Distance-Angle 消息
 */
void tcpDataLoop(ros::Publisher& raw_pub, 
                ros::Publisher& point_pub,
                ros::Publisher& da_pub) {
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

            // 解析有效数据包
            parseAndPublishData(raw_data.substr(0, last_newline), point_pub, da_pub);
            remaining_data = raw_data.substr(last_newline + 1);
        } else {
            remaining_data = raw_data;
            ROS_DEBUG("Buffering incomplete packet");
        }
        memset(buffer, 0, sizeof(buffer));
    }
}

/**
 * @brief 解析原始数据并发布 ROS 消息
 *  解析 XYZ 坐标和 Distance-Angle 数据，并分别发布到对应话题
 *  @param data 原始数据字符串
 *  @param point_pub geometry_msgs::PointStamped 发布器
 *  @param da_pub pipeline_robot::DistanceAngle 发布器
 */
void parseAndPublishData(const std::string& data, 
                        ros::Publisher& point_pub,
                        ros::Publisher& da_pub) {
    std::istringstream iss(data);
    std::string line;
    geometry_msgs::PointStamped point;
    point.header.frame_id = "sensor_frame";
    bool has_coordinates = false;

    pipeline_robot::DistanceAngle da_msg;
    da_msg.header.stamp = ros::Time::now();

    while (std::getline(iss, line)) {
        if (line.empty()) continue;
        ROS_DEBUG_STREAM("Processing: " << line);

        try {
            // 解析XYZ坐标
            if (line.find("X=") == 0) {
                point.point.x = std::stof(line.substr(2));
                has_coordinates = true;
            } 
            else if (line.find("Y=") == 0) {
                point.point.y = std::stof(line.substr(2));
                has_coordinates = true;
            } 
            else if (line.find("Z=") == 0) {
                point.point.z = std::stof(line.substr(2));
                if (has_coordinates) {
                    point.header.stamp = ros::Time::now();
                    point_pub.publish(point);
                    ROS_DEBUG_STREAM("Published point: " << 
                                    point.point.x << ", " << 
                                    point.point.y << ", " << 
                                    point.point.z);
                    has_coordinates = false;
                }
            }
            // 解析Distance和Angle
            else if (line.find("D=") == 0 && line.find("A=") != std::string::npos) {
                size_t a_pos = line.find("A=");
                da_msg.distance = std::stof(line.substr(2, a_pos - 2));
                da_msg.angle = std::stof(line.substr(a_pos + 2));
                da_msg.header.stamp = ros::Time::now();
                da_pub.publish(da_msg);
                ROS_DEBUG_STREAM("Published DA: D=" << da_msg.distance << ", A=" << da_msg.angle);
            }
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Parse error: " << e.what() << " in line: " << line);
        }
    }
}

/**
 * @brief 处理 TCP 断开连接
 *  尝试重新连接服务器，如果达到最大重连次数则关闭 ROS 节点
 *  @param max_reconnect 最大重连次数
 *  @param interval 重连间隔时间（秒）
 */
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

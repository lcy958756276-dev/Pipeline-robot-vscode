/**
 * @file tcp_current_node.cpp
 * @brief ROS节点，通过TCP连接接收电流数据，并将解析后的数据发布为Float32消息
 *
 * 功能：
 * - 通过TCP客户端连接指定IP和端口，接收传感器/控制器发送的电流数据。
 * - 解析数据中的 c0~c4 五个通道，并按公式转换为实际电流值。
 * - 发布电流值到对应 ROS 话题。
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sstream>
#include <string>

// ------------------ 全局发布器 ------------------
ros::Publisher pub_c0, pub_c1, pub_c2, pub_c3, pub_c4;

/**
 * @brief 从输入字符串中提取指定前缀后的数值
 * @param input 输入字符串
 * @param prefix 前缀，例如 "c0:"
 * @return std::string 提取的数值字符串，如果未找到返回空字符串
 */
std::string extractData(const std::string& input, const std::string& prefix) {
    std::size_t pos = input.find(prefix);
    if (pos == std::string::npos) return "";
    std::string num;
    for (std::size_t i = pos + prefix.size(); i < input.size(); ++i) {
        if (input[i] == 'c' || input[i] == ' ' || input[i] == '\n') break;
        num += input[i];
    }
    return num;
}

/**
 * @brief 检查字符串是否是有效整数
 * @param str 输入字符串
 * @return true 是整数
 * @return false 否
 */
bool isValidInteger(const std::string& str) {
    if (str.empty()) return false;
    char *p;
    strtol(str.c_str(), &p, 10);
    return (*p == 0);
}

/**
 * @brief 解析一行TCP数据并发布到对应ROS话题
 * @param line 输入字符串
 *
 * 功能：
 * - 解析 c0~c4 五个通道数据。
 * - 按照公式转换为实际电流值：
 *   - c0,c1: 5*value/4095*1.55
 *   - c2: 5*(value-2048)/2048
 *   - c3,c4: 5*value/4095*0.155
 * - 发布到对应 Float32 话题，并打印日志。
 */
void parseAndPublish(const std::string& line) {
    std::string c0 = extractData(line, "c0:");
    std::string c1 = extractData(line, "c1:");
    std::string c2 = extractData(line, "c2:");
    std::string c3 = extractData(line, "c3:");
    std::string c4 = extractData(line, "c4:");

    if (isValidInteger(c0)) {
        std_msgs::Float32 msg;
        msg.data = 5 * (std::stof(c0)) / 4095 * 1.55;
        pub_c0.publish(msg);
        ROS_INFO("c0 = %.3f", msg.data);
    }
    if (isValidInteger(c1)) {
        std_msgs::Float32 msg;
        msg.data = 5 * (std::stof(c1)) / 4095 * 1.55;
        pub_c1.publish(msg);
        ROS_INFO("c1 = %.3f", msg.data);
    }
    if (isValidInteger(c2)) {
        std_msgs::Float32 msg;
        msg.data = 5 * (std::stof(c2) - 2048) / 2048;
        pub_c2.publish(msg);
        ROS_INFO("c2 = %.3f", msg.data);
    }
    if (isValidInteger(c3)) {
        std_msgs::Float32 msg;
        msg.data = 5 * (std::stof(c3)) / 4095 * 0.155;
        pub_c3.publish(msg);
        ROS_INFO("c3 = %.3f", msg.data);
    }
    if (isValidInteger(c4)) {
        std_msgs::Float32 msg;
        msg.data = 5 * (std::stof(c4)) / 4095 * 0.155;
        pub_c4.publish(msg);
        ROS_INFO("c4 = %.3f", msg.data);
    }
}

/**
 * @brief TCP客户端主循环
 * @param ip 服务器IP
 * @param port 服务器端口
 *
 * 功能：
 * - 创建TCP socket并连接服务器。
 * - 循环接收数据，按行解析。
 * - 对不完整行进行缓存处理，保证数据完整。
 * - 调用 parseAndPublish 将解析结果发布到 ROS。
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
 * @brief 主函数，初始化ROS节点并启动TCP循环
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "tcp_current_node");
    ros::NodeHandle nh;

    std::string ip;
    int port;
    nh.param<std::string>("ip", ip, "192.168.8.11");
    nh.param<int>("port", port, 12345);

    // 初始化发布器
    pub_c0 = nh.advertise<std_msgs::Float32>("current_data_cs0", 10);
    pub_c1 = nh.advertise<std_msgs::Float32>("current_data_cs1", 10);
    pub_c2 = nh.advertise<std_msgs::Float32>("current_data_cs2", 10);
    pub_c3 = nh.advertise<std_msgs::Float32>("current_data_cs3", 10);
    pub_c4 = nh.advertise<std_msgs::Float32>("current_data_cs4", 10);

    tcpLoop(ip, port);
    return 0;
}

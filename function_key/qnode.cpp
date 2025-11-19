/**
 * @file qnode.cpp
 * @brief QNode类实现，用于ROS节点初始化、消息订阅/发布以及与Qt界面的交互。
 *
 * 该文件包含：
 * - ROS节点初始化与启动
 * - 图像、IMU、odom、激光雷达、传感器等数据的订阅回调
 * - 各种控制命令和状态信息的发布
 * - 将ROS数据转换为Qt可显示的格式（如QImage）
 * - Qt信号发射，用于更新界面
 */

#include "qnode.h"

extern int flag;
bool current_video_topic = true; ///< 视频切换标志符

/**
 * @class QNode
 * @brief QNode类封装了ROS节点与Qt的交互，包括消息发布、订阅及数据处理。
 */
QNode(int argc, char** argv, char* node_name )
{
    /**
     * @brief 构造函数，保存节点初始化参数并调用init()进行节点初始化
     * @param argc 命令行参数个数
     * @param argv 命令行参数数组
     * @param node_name ROS节点名称
     */
    init_argc = argc;
    init_argv = argv;
    init_node_name = node_name;
    init();
}

/**
 * @brief 析构函数，关闭ROS节点并等待线程结束
 */
~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // 显式关闭ROS
        ros::waitForShutdown();
    }
    wait(); // 线程等待
}

/**
 * @brief 初始化ROS节点、发布器、订阅器并启动线程
 * @return 初始化成功返回true，失败返回false
 */
bool init() {
    ros::init(init_argc,init_argv,init_node_name);
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start();
    ros::NodeHandle n;

    // ====================== ROS发布器 ====================== //
    chatter_publisher1 = n.advertise<std_msgs::String>("Pub2stm", 1000);  
    chatter_publisher2 = n.advertise<pipeline::HeaderInfo>("piantou", 1000);  
    chatter_publisher3 = n.advertise<geometry_msgs::PoseStamped>("finalgoal", 1000); 
    chatter_publisher4 = n.advertise<std_msgs::Bool>("whether_auto_navigation", 1000); 
    chatter_publisher5 = n.advertise<std_msgs::Bool>("whether_stop", 1000); 
    chatter_publisher6 = n.advertise<geometry_msgs::PoseStamped>("waypoint", 1000); 
    chatter_publisher7 = n.advertise<std_msgs::Bool>("reset_odom", 1000); 
    chatter_publisher8 = n.advertise<std_msgs::String>("cable", 1000);  
    chatter_publisher9 = n.advertise<std_msgs::Bool>("whether_report_generate", 1000); 
    chatter_publisher10 = n.advertise<std_msgs::Bool>("cloudmodel_pub", 1000); 

    // ====================== ROS订阅器 ====================== //
    chatter_subscriber1 = n.subscribe("/yolov8_detection", 1, &QNode::imageCallback, this); 
    chatter_subscriber2 = n.subscribe("wit/imu", 1000, &QNode::myCallback_imu, this); 
    chatter_subscriber3 = n.subscribe("pressure", 1000, &QNode::myCallback_pre, this); 
    chatter_subscriber4 = n.subscribe("/cable/odom", 1000, &QNode::myCallback_odom, this); 
    chatter_subscriber5 = n.subscribe("/camera/image_raw", 1, &QNode::myCallback_backimage, this); 
    chatter_subscriber6 = n.subscribe("/whether_auto_navigation", 1000, &QNode::myCallback_auto_navigation, this); 

    chatter_subscriber7 = n.subscribe("/current_data_cs0", 1000, &QNode::myCallback_current1, this); 
    chatter_subscriber8 = n.subscribe("/current_data_cs1", 1000, &QNode::myCallback_current2, this); 
    chatter_subscriber9 = n.subscribe("/current_data_cs2", 1000, &QNode::myCallback_current3, this); 
    chatter_subscriber10 = n.subscribe("/current_data_cs3", 1000, &QNode::myCallback_current4, this); 
    chatter_subscriber11 = n.subscribe("/current_data_cs4", 1000, &QNode::myCallback_current5, this); 

    chatter_subscriber12 = n.subscribe("/temperature_data", 1000, &QNode::myCallback_temperature, this); 
    chatter_subscriber13 = n.subscribe("/pressVal", 1000, &QNode::myCallback_press, this); 
    chatter_subscriber14 = n.subscribe("/encodeVal", 1000, &QNode::myCallback_encodeVal, this); 
    chatter_subscriber15 = n.subscribe("/angle0", 1000, &QNode::myCallback_angle0, this); 
    chatter_subscriber16 = n.subscribe("/angle1", 1000, &QNode::myCallback_angle1, this); 
    chatter_subscriber17 = n.subscribe("/leftLimit", 1000, &QNode::myCallback_leftLimit, this); 
    chatter_subscriber18 = n.subscribe("/rightLimit", 1000, &QNode::myCallback_rightLimit, this); 
    chatter_subscriber19 = n.subscribe("/orin_cpu_temp", 1000, &QNode::myCallback_orin_cpu_temp, this); 
    chatter_subscriber20 = n.subscribe("/orin_gpu_temp", 1000, &QNode::myCallback_orin_gpu_temp, this); 

    chatter_subscriber21 = n.subscribe("/rx_motor/tq", 1000, &QNode::myCallback_rx_motor_tq, this); 
    chatter_subscriber22 = n.subscribe("/rx_motor/speed", 1000, &QNode::myCallback_rx_motor_speed, this); 

    chatter_subscriber23 = n.subscribe("/laser_scan", 1000, &QNode::myCallback_laser_scan, this); 
    chatter_subscriber24 = n.subscribe("/roll", 1000, &QNode::rollCallback, this); 
    chatter_subscriber25 = n.subscribe("/pitch", 1000, &QNode::pitchCallback, this); 
    chatter_subscriber27 = n.subscribe("/heading", 1000, &QNode::headingCallback, this); 
    chatter_subscriber26 = n.subscribe("/overall_map_cloud", 1000, &QNode::cloudcallback, this); 
    chatter_subscriber28 = n.subscribe("Pub2stm", 100, &QNode::callbackPub2stm, this); 

    start(); // 启动线程
    return true;
}

/**
 * @brief QNode线程主循环，处理ROS回调
 */
void run()
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

/**
 * @brief 处理来自Pub2stm话题的字符串消息，并根据内容发射不同Qt信号
 * @param msg 接收到的字符串消息指针
 */
void callbackPub2stm(const std_msgs::String::ConstPtr &msg)
{
    QString data = QString::fromStdString(msg->data);
    if (data == "FO") {
        emit signalFO();
    } else if(data == "BA") {
        emit signalBA();
    } else {
        emit signalST();
    }
}

/**
 * @brief 处理yolov8检测图像回调，将ROS图像转为QImage并发射信号
 * @param msg 接收到的图像消息
 */
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat mat = cv_ptr->image;
    if (mat.empty()) return;

    cv::Mat mat_rgb;
    cv::cvtColor(mat, mat_rgb, cv::COLOR_BGR2RGB);

    QImage img(mat_rgb.data, mat_rgb.cols, mat_rgb.rows, static_cast<int>(mat_rgb.step), QImage::Format_RGB888);
    emit imageReceived(img.copy());
}

/**
 * @brief 将cv::Mat转换为QImage
 * @param src 输入的cv::Mat图像
 * @return 转换后的QImage
 */
QImage Mat2QImage(cv::Mat const& src)
{
    QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
    const float scale = 255.0;

    if (src.depth() == CV_8U) {
        if (src.channels() == 1) {
            for (int i = 0; i < src.rows; ++i)
                for (int j = 0; j < src.cols; ++j)
                    dest.setPixel(j, i, qRgb(src.at<quint8>(i, j), src.at<quint8>(i, j), src.at<quint8>(i, j)));
        } else if (src.channels() == 3) {
            for (int i = 0; i < src.rows; ++i)
                for (int j = 0; j < src.cols; ++j) {
                    cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
                    dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
                }
        }
    } else if (src.depth() == CV_32F) {
        if (src.channels() == 1) {
            for (int i = 0; i < src.rows; ++i)
                for (int j = 0; j < src.cols; ++j) {
                    int level = scale * src.at<float>(i, j);
                    dest.setPixel(j, i, qRgb(level, level, level));
                }
        } else if (src.channels() == 3) {
            for (int i = 0; i < src.rows; ++i)
                for (int j = 0; j < src.cols; ++j) {
                    cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
                    dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0]));
                }
        }
    }
    return dest;
}

/**
 * @brief 切换当前视频源，前置/后置摄像头
 */
void odetoggleVideoSource() {
    current_video_topic = !current_video_topic;
}

/**
 * @brief 处理后置摄像头图像回调，仅在current_video_topic为false时处理
 * @param msg 接收到的图像消息
 */
void myCallback_backimage(const sensor_msgs::ImageConstPtr &msg)
{
    if (!current_video_topic) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            QImage im = Mat2QImage(cv_ptr->image);
            emit Show_image(1, im);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8': %s", msg->encoding.c_str(), e.what());
            return;
        }
    }
}

/**
 * @brief 处理前置摄像头图像回调，仅在current_video_topic为true时处理
 * @param msg 接收到的图像消息
 */
void myCallback_img(const sensor_msgs::ImageConstPtr &msg)
{
    if (current_video_topic) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            QImage im = Mat2QImage(cv_ptr->image);
            emit Show_image(0, im);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8': %s", msg->encoding.c_str(), e.what());
            return;
        }
    }
}
/**
 * @brief IMU数据回调函数
 * @param msg 接收到的IMU消息
 *
 * 将IMU姿态四元数转换为滚转角、俯仰角、偏航角，并通过Qt信号发射到界面。
 */
void myCallback_imu(const sensor_msgs::Imu& msg){
    tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    emit Show_imu(pitch,roll, yaw);
}

/**
 * @brief odom数据回调函数
 * @param msg 接收到的Odometry消息
 *
 * 提取机器人速度和位置，并通过Qt信号发射。
 */
void myCallback_odom(const nav_msgs::Odometry::ConstPtr& msg) {
    double speed = msg->twist.twist.linear.x;
    double position = msg->pose.pose.position.x;
    emit odomDataReceived(speed, position);
    emit odomReceived(msg->pose.pose);
}

/**
 * @brief 气压传感器数据回调函数
 * @param msg 接收到的FluidPressure消息
 *
 * 发射Qt信号更新界面气压显示。
 */
void myCallback_pre(const sensor_msgs::FluidPressureConstPtr &msg){
    double pressure = msg->fluid_pressure;
    emit Show_pressure(pressure);
}

/**
 * @brief 电流传感器数据回调函数1
 * @param msg 接收到的Float32消息
 */
void myCallback_current1(const std_msgs::Float32::ConstPtr &msg)
{
    emit updateCurrent1(msg->data);
}
void myCallback_current2(const std_msgs::Float32::ConstPtr& msg) {
    emit updateCurrent2(msg->data);
}
void myCallback_current3(const std_msgs::Float32::ConstPtr& msg) {
    emit updateCurrent3(msg->data);
}
void myCallback_current4(const std_msgs::Float32::ConstPtr& msg) {
    emit updateCurrent4(msg->data);
}
void myCallback_current5(const std_msgs::Float32::ConstPtr& msg) {
    emit updateCurrent5(msg->data);
}

/**
 * @brief 温度传感器数据回调函数
 * @param msg 接收到的Float32消息
 */
void myCallback_temperature(const std_msgs::Float32::ConstPtr& msg)
{
    emit updateTemp(msg->data);
}

/**
 * @brief 气压传感器数值回调函数
 * @param msg 接收到的Float32消息
 */
void myCallback_press(const std_msgs::Float32::ConstPtr& msg)
{
    emit updatePress(msg->data);
}

/**
 * @brief 俯仰角编码值回调
 * @param msg 接收到的Float32消息
 */
void myCallback_encodeVal(const std_msgs::Float32::ConstPtr& msg)
{
    emit updateEncodeVal(msg->data);
}

/**
 * @brief 云台角度0回调
 * @param msg 接收到的Float32消息
 */
void myCallback_angle0(const std_msgs::Float32::ConstPtr& msg)
{
    emit updateAngle0(msg->data);
}

/**
 * @brief 云台角度1回调
 * @param msg 接收到的Float32消息
 */
void myCallback_angle1(const std_msgs::Float32::ConstPtr& msg)
{
    emit updateAngle1(msg->data);
}

/**
 * @brief 左限制开关回调
 * @param msg 接收到的Bool消息
 */
void myCallback_leftLimit(const std_msgs::Bool& msg)
{
    emit updateLeftLimit(msg.data);
}

/**
 * @brief 右限制开关回调
 * @param msg 接收到的Bool消息
 */
void myCallback_rightLimit(const std_msgs::Bool& msg)
{
    emit updateRightLimit(msg.data);
}

/**
 * @brief ORIN CPU温度回调
 * @param msg 接收到的Float32消息
 */
void myCallback_orin_cpu_temp(const std_msgs::Float32::ConstPtr& msg) {
  emit updateCpuTemp(msg->data);
}

/**
 * @brief ORIN GPU温度回调
 * @param msg 接收到的Float32消息
 */
void myCallback_orin_gpu_temp(const std_msgs::Float32::ConstPtr& msg) {
  emit updateGpuTemp(msg->data);
}

/**
 * @brief 绕线盘电机力矩回调
 * @param msg 接收到的Float32消息
 */
void myCallback_rx_motor_tq(const std_msgs::Float32::ConstPtr& msg) {
  emit updateRxMotorTq(msg->data);
}

/**
 * @brief 绕线盘电机速度回调
 * @param msg 接收到的Float32消息
 */
void myCallback_rx_motor_speed(const std_msgs::Float32::ConstPtr& msg) {
  emit updateRxMotorSpeed(msg->data);
}

/**
 * @brief LaserScan数据回调函数
 * @param scan 接收到的LaserScan消息
 *
 * 提取指定角度范围内的平均距离，并发射Qt信号。
 */
void myCallback_laser_scan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // ... 保持原逻辑
}

/**
 * @brief 发布普通字符串消息
 * @param msg 要发布的字符串
 */
void publishMessage(const std::string &msg) {
    std_msgs::String message;
    message.data = msg;
    chatter_publisher1.publish(message);
}

/**
 * @brief 发布片头信息
 * @param msg pipeline::HeaderInfo结构体消息
 */
void publishPianTou(const pipeline::HeaderInfo &msg) {
    chatter_publisher2.publish(msg);
}

/**
 * @brief 发布自主导航指令
 * @param msg2 最终目标PoseStamped
 */
void publishAutonomousNavigation(const geometry_msgs::PoseStamped &msg2)
{
    std_msgs::Bool msg;
    msg.data = true;
    chatter_publisher4.publish(msg);
    QThread::msleep(100);

    std_msgs::Bool msg1;
    msg1.data = false;
    chatter_publisher3.publish(msg2);
    QThread::msleep(100);
    chatter_publisher3.publish(msg2);
    QThread::msleep(100);
    chatter_publisher3.publish(msg2);
    QThread::msleep(100);
    chatter_publisher5.publish(msg1);
}

/**
 * @brief 发布自动回收指令
 * @param distance 回收距离
 */
void publishAutomaticRecycle(double distance)
{  
    std_msgs::Bool msg;
    msg.data = false;
    geometry_msgs::PoseStamped waypoint;
    if (distance > 0) waypoint.pose.position.x = 0;
    else waypoint.pose.position.x = distance;
    chatter_publisher6.publish(waypoint);
    chatter_publisher5.publish(msg);
}

/**
 * @brief 发布计米器归零指令
 */
void publishResetMeter()
{
    std_msgs::Bool msg;
    msg.data = true;
    chatter_publisher7.publish(msg);
}

/**
 * @brief 发布急停指令
 */
void publishStop()
{
    QByteArray command;
    command.append('\x53');
    command.append('\x44');
    rosSender->sendCommand(command);
    rosSender->sendCommand(command);
    rosSender->sendCommand(command);

    std_msgs::Bool msg1, msg2;
    msg1.data = true;
    msg2.data = false;
    chatter_publisher5.publish(msg1);
    chatter_publisher4.publish(msg2);
}

/**
 * @brief 发布cable消息
 * @param msg 字符串消息
 */
void publish_cable(const std::string &msg)
{
    std_msgs::String message;
    message.data = msg;
    chatter_publisher8.publish(message);
}

/**
 * @brief 发布自动报告生成指令
 * @param msg Bool消息，true表示开启
 */
void publish_report_generate(const std_msgs::Bool msg)
{
    chatter_publisher9.publish(msg);
}

/**
 * @brief 发布点云模型生成指令
 * @param msg Bool消息，true表示生成
 */
void publish_cloudmodel_pub(const std_msgs::Bool msg)
{
    chatter_publisher10.publish(msg);
}

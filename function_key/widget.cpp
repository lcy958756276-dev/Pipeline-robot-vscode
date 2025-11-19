/** @class Widget */

/**
 * @file widget.cpp
 * @brief Widget 类的实现文件
 *
 * 本文件实现了系统的主界面逻辑，包括：
 *
 * ### 1. UI 界面初始化
 * - 初始化主窗口、控件、样式和布局
 * - 初始化子窗口（如电机速度测试窗口 MotorSpeedTest）
 * - 配置标签、按钮、视频显示区域、状态栏等 UI 元素
 *
 * ### 2. RTSP 视频解码与显示
 * - 使用 RtspDecoder / FfplayWrapper 解码 RTSP 视频流
 * - 显示当前帧至 QLabel（label_35）
 * - 实现摄像头切换、延迟显示、截图、视频录制等功能
 *
 * ### 3. ROS 通信模块
 * - 订阅多个 ROS Topic（如 IMU、里程、激光、传感器等）
 * - 发布控制命令（自动导航、启动/停止、复位、回收等）
 * - 通过回调槽更新 UI 显示与系统状态变量
 *
 * ### 4. 传感器数据处理
 * - 经度、纬度、IMU、里程、激光测距等数据解析
 * - 深度测量、延迟计算、姿态角校准
 * - 对传感器数据进行平均、滤波、转换与显示
 *
 * ### 5. 下位机控制与远程 SSH 操作
 * - 一键启动下位机程序（run_robot.sh）
 * - 远程关闭 / 重启机器人（通过 sshpass 执行 kill 或 shutdown）
 * - 检测与提示远程执行结果
 *
 * ### 6. 视频录制功能
 * - 开始/停止录制 RTSP 视频流
 * - 自动生成时间戳视频文件名
 * - 录制完成后弹窗提示（onRecordingFinished）
 *
 * ### 7. 各类控制按钮槽函数
 * - 截图、切换摄像头、开始/结束深度测量
 * - 初始化姿态角校准
 * - 打开电机速度测试界面
 * - UI 按钮状态联动与动态样式切换
 *
 * ---
 *
 * 本文件实现了主界面从 UI、ROS 到视频和控制所有核心业务逻辑，
 * 是整个管道机器人可视化与交互系统的中心模块。
 */


#include "widget.h"
#include "ui_widget.h"
#include "ros_tcp_sender.h"
#include "ros/ros.h"
#include<QThread>
#include "videorecorder.h"

int flag=0;

/**
 * @brief Widget类构造函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @param parent 父QWidget指针
 *
 * 初始化UI界面、ROS节点、TCP发送器、Modbus、GPS、视频解码器和定时器。
 * 绑定信号和槽，包括传感器数据、视频显示、摄像头切换、按钮控制。
 */
Widget(int argc, char** argv, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget),
    settingWidgetInstance(nullptr),
    jiantuWidgetInstance(nullptr),
    node(argc, argv, "node_test"),
    motorSpeedTestWidgetInstance(nullptr)
{
    ui->setupUi(this);
    tempArgc = argc;
    tempArgv = argv;

    rosSender = new RosTcpSender(this);
    my_modbus = new modbus();
    my_modbus->mod_init();
    qDebug() << "aaa";

    my_gps_get = new GpsGet();
    my_gps_get->gps_start();

    decoder_ = new RtspDecoder(this);
    connect(decoder_, &RtspDecoder::frameDecoded, this, &Widget::onFrameDecoded);
    decoder_->setUrl("rtsp://admin:123456@192.168.8.9/Streaming/Channels/2");
    decoder_->start();
    connect(decoder_, &RtspDecoder::connectionLost, this, [this](){
        QTimer::singleShot(1000, this, [this](){
            on_pushButton_3_clicked();  // 自动触发重连
        });
    });

    ui->label_7->show();
    ui->label_35->hide();

    m_recorder = new VideoRecorder(this);
    connect(m_recorder, &VideoRecorder::recordingFinished, this, &Widget::onRecordingFinished);
    m_currentCamera = 0; // 默认第一个摄像头

    // 绑定ROS信号和槽
    connect(&node, &QNode::signalFO, this, &Widget::slotFOTriggered);
    connect(&node, &QNode::signalBA, this, &Widget::slotBATriggered);
    connect(&node, &QNode::signalST, this, &Widget::slotSTTriggered);

    connect(&node, &QNode::imageReceived, this, &Widget::updateImage);
    connect(my_gps_get, &GpsGet::LondataReceived, this, &Widget::onLonRecived);
    connect(my_gps_get, &GpsGet::LatdataReceived, this, &Widget::onLatRecived);

    // 绑定UI显示信号
    connect(&node, SIGNAL(Show_image(int,QImage)) ,this,SLOT(slot_show_image(int, QImage)));
    connect(&node, SIGNAL(Show_imu(double,double,double)) ,this,SLOT(slot_show_imu(double,double,double)));
    connect(&node, SIGNAL(odomDataReceived(double,double)), this, SLOT(updateOdomLabels(double,double)));
    connect(&node, SIGNAL(Show_auto_navigation(bool)), this, SLOT(update_auto_navigation(bool)));

    connect(&node, SIGNAL(updateCurrent1(float)), this, SLOT(displayCurrentLeftMotor(float)));
    connect(&node, SIGNAL(updateCurrent2(float)), this, SLOT(displayCurrentRightMotor(float)));
    connect(&node, SIGNAL(updateCurrent3(float)), this, SLOT(displayCurrentLiftMotor(float)));
    connect(&node, SIGNAL(updateCurrent4(float)), this, SLOT(displayCurrentPitchMotor(float)));
    connect(&node, SIGNAL(updateCurrent5(float)), this, SLOT(displayCurrentRotationMotor(float)));

    connect(&node, SIGNAL(updateEncodeVal(float)), this, SLOT(update_encodeVal(float)));
    connect(&node, SIGNAL(updateAngle0(float)), this, SLOT(update_angle0(float)));
    connect(&node, SIGNAL(updateAngle1(float)), this, SLOT(update_angle1(float)));
    connect(&node, SIGNAL(updateLeftLimit(bool)), this, SLOT(update_leftLimit(bool)));
    connect(&node, SIGNAL(updateRightLimit(bool)), this, SLOT(update_rightLimit(bool)));

    connect(&node, SIGNAL(updateRxMotorTq(float)), this, SLOT(update_rx_motor_tq(float)));
    connect(&node, SIGNAL(updateRxMotorSpeed(float)), this, SLOT(update_rx_motor_speed(float)));

    connect(&node, SIGNAL(updateTopLaser(float)), this, SLOT(update_top_laser(float)));
    connect(&node, SIGNAL(updateDownLaser(float)), this, SLOT(update_down_laser(float)));

    // 绑定定时器槽函数
    connect(leftLimitTimer, &QTimer::timeout, this, &Widget::checkLeftLimit);
    connect(rightLimitTimer, &QTimer::timeout, this, &Widget::checkRightLimit);
    connect(resetTimer, &QTimer::timeout, this, &Widget::resetAll);
    connect(rotateTimer, &QTimer::timeout, this, &Widget::rotateTimerSlot);
    connect(rxTimer, &QTimer::timeout, this, &Widget::rxTimerSlot);

    loadSettings();
    remoteProcess = new QProcess(this);

    // 启动下位机
    on_start_robot_button_clicked();
}

/**
 * @brief Widget析构函数
 *
 * 停止Modbus，关闭文件，释放UI和动态分配的子窗口指针
 */
Widget()
{
    my_modbus->stopLine();
    delete ui;
    delete settingWidgetInstance;
    delete jiantuWidgetInstance;
    file_save_imu.close();
}

/**
 * @brief 视频解码帧回调函数
 * @param image 解码得到的QImage帧
 *
 * 更新显示标签，并在录制状态下将帧添加到VideoRecorder
 */
void onFrameDecoded(const QImage &image) {
    QSize targetSize(ui->label_7->width(), ui->label_7->height());
    QPixmap pix = QPixmap::fromImage(image.scaled(targetSize, Qt::KeepAspectRatio));
    ui->label_7->setPixmap(pix);

    m_currentFrame = image;
    if (m_recorder->isRecording() && m_currentCamera == 0) {
        m_recorder->addFrame(image);
    }
}

/**
 * @brief 第二路视频帧回调函数
 * @param image 解码得到的QImage帧
 */
void onFrameDecoded1(const QImage &image) {
    QSize targetSize(ui->label_35->width(), ui->label_35->height());
    QPixmap pix = QPixmap::fromImage(image.scaled(targetSize, Qt::KeepAspectRatio));
    ui->label_35->setPixmap(pix);

    m_currentFrame = image;
    if (m_recorder->isRecording() && m_currentCamera == 1) {
        m_recorder->addFrame(image);
    }
}

/**
 * @brief 重写窗口关闭事件
 * @param event QCloseEvent指针
 *
 * 关闭机器人连接并执行窗口关闭
 */
void closeEvent(QCloseEvent *event) {
    qDebug() << "窗口即将关闭，执行清理操作...";
    on_quit_robot_button_clicked();
    event->accept();
}

/**
 * @brief rxTimer定时器槽函数
 *
 * 定时发送cable消息“TQ”和“SP”
 */
void rxTimerSlot()
{
    t_tr++;
    if(t_tr == 20) {
        node.publish_cable("TQ");
    }
    if(t_tr >= 40) {
        node.publish_cable("SP");
        t_tr = 0;
    }
}

/**
 * @brief 摄像头切换按钮槽函数
 *
 * 切换显示摄像头和m_currentCamera标志
 */
void on_pushButton_4_clicked()
{
    flag = !flag;
    m_currentCamera = !m_currentCamera;

    if(!flag) {
        ui->label_7->show();
        ui->label_35->hide();
    } else {
        ui->label_35->show();
        ui->label_7->hide();
    }
}

/**
 * @brief IMU数据槽函数
 * @param pitch 俯仰角 (rad)
 * @param roll 横滚角 (rad)
 * @param yaw 偏航角 (rad)
 *
 * 更新UI显示，当倾角超过30度时，执行急停并提示
 */
void slot_show_imu(double pitch, double roll, double yaw)
{
    QString pitchText = QString::number(pitch * 180.0 /M_PI + imu_pitch_error, 'f', 2)+"°";
    QString rollText = QString::number(roll * 180.0 /M_PI + imu_roll_error, 'f', 2)+"°";
    QString yawText = QString::number(yaw * 180.0 /M_PI, 'f', 2)+"°";

    ui->label_5->setText(rollText);
    ui->label_6->setText(pitchText);
    ui->label_yaw->setText(yawText);

    if (qAbs(pitch * 180.0 /M_PI + imu_pitch_error) > 30.0 || qAbs(roll * 180.0 /M_PI + imu_roll_error) > 30.0)
    {
        if(imu_flag) return;
        node.publishStop();
        on_start_record_button_clicked();
        imu_flag = true;
        QMessageBox::warning(this, tr("警告"), tr("检测到倾角超过30度，请注意！"), QMessageBox::Ok);
    } else {
        imu_flag = false;
    }
}

/**
 * @brief 左电机电流显示槽函数
 * @param value 电流值
 *
 * 超过限制时执行急停并提示
 */
void displayCurrentLeftMotor(float value) {
    ui->labelCurrentLeftMotor->setText(QString::number(value, 'f', 4));
    if (qAbs(value) > current_limit) {
        if(currentL_limit_flag) return;
        node.publishStop();
        on_start_record_button_clicked();
        currentL_limit_flag = true;
    } else {
        currentL_limit_flag = false;
    }
}

/**
 * @brief 显示右电机电流
 * @param value 电流值
 *
 * 超过限制时触发保护逻辑
 */
bool currentR_limit_flag = false;
void displayCurrentRightMotor(float value) {
    ui->labelCurrentRightMotor->setText(QString::number(value, 'f', 4));
    if (qAbs(value) > current_limit) {
        if(currentR_limit_flag) return;
        // 可加急停逻辑
        currentR_limit_flag = true;
    } else {
        currentR_limit_flag = false;
    }
}

/**
 * @brief 显示升降台电机电流（C0）
 * @param value 电流值
 *
 * 超过限制时触发保护逻辑，停止旋转和重置定时器
 */
bool currentsj_limit_flag = false;
void displayCurrentLiftMotor(float value) {
    ui->labelCurrentLiftMotor->setText(QString::number(value, 'f', 4));
    if (qAbs(value) > currentsj_limit) {
        if(currentsj_limit_flag) return;
        cameraButtonReleased();
        rotateTimer->stop();
        state_rotate = -1;
        state = 3;
        resetTimer->stop();
        currentsj_limit_flag = true;
    } else {
        currentsj_limit_flag = false;
    }
}

/**
 * @brief 显示俯仰电机电流（C1）
 * @param value 电流值
 *
 * 超过限制时触发保护逻辑
 */
bool currentfy_limit_flag = false;
void displayCurrentPitchMotor(float value) {
    ui->labelCurrentPitchMotor->setText(QString::number(value, 'f', 4));
    if (qAbs(value) > currentfy_limit) {
        if(currentfy_limit_flag) return;
        cameraButtonReleased();
        rotateTimer->stop();
        state_rotate = -1;
        state = 3;
        resetTimer->stop();
        currentfy_limit_flag = true;
    } else {
        currentfy_limit_flag = false;
    }
}

/**
 * @brief 显示旋转电机电流（C2）
 * @param value 电流值
 *
 * 超过限制时触发保护逻辑
 */
bool currentxz_limit_flag = false;
void displayCurrentRotationMotor(float value) {
    ui->labelCurrentLiftMotor->setText(QString::number(value, 'f', 4));
    if (qAbs(value) > currentxz_limit) {
        if(currentxz_limit_flag) return;
        cameraButtonReleased();
        rotateTimer->stop();
        state_rotate = -1;
        state = 3;
        resetTimer->stop();
        currentxz_limit_flag = true;
    } else {
        currentxz_limit_flag = false;
    }
}

/**
 * @brief 更新编码器值
 * @param value 编码器数值
 */
void update_encodeVal(float value) {
    encodeVal = value;
}

/**
 * @brief 控制角度0顺时针旋转（SSZ）
 */
void angle0_SSZ() {
    QByteArray command;
    command.append('\x59'); command.append('\x4F'); command.append('\x55'); 
    command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
}

/**
 * @brief 控制角度0逆时针旋转（NSZ）
 */
void angle0_NSZ() {
    QByteArray command;
    command.append('\x5A'); command.append('\x55'); command.append('\x4F'); 
    command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
}

/**
 * @brief 控制角度1升起（NSZ）
 */
void angle1_up() {
    QByteArray command;
    command.append('\x4E'); command.append('\x53'); command.append('\x5A');
    command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
}

/**
 * @brief 控制角度1下降
 */
void angle1_down() {
    QByteArray command;
    command.append('\x44'); command.append('\x4F'); command.append('\x57'); 
    command.append('\x4E'); command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
}

/**
 * @brief 保存angle0_round和angle1_round到文件
 */
void saveSettings() {
    QFile file(settingsFile);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);
        out << "angle0_round=" << angle0_round << "\n";
        out << "angle1_round=" << angle1_round << "\n";
        file.close();
    } else {
        qDebug() << "无法打开文件进行保存";
    }
}

/**
 * @brief 从文件加载angle0_round和angle1_round
 */
void loadSettings() {
    QFile file(settingsFile);
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream in(&file);
        QString line1 = in.readLine();
        QString line2 = in.readLine();
        if (!line1.isEmpty()) angle0_round = line1.split('=').last().toInt();
        if (!line2.isEmpty()) angle1_round = line2.split('=').last().toInt();
        file.close();
    } else {
        qDebug() << "无法打开文件进行读取";
    }
}

/**
 * @brief 旋转到指定目标角度
 * @param angleType 0表示角度0，1表示角度1
 * @param targetAngleTemp 目标角度
 */
void rotateToTargetAngle(int angleType, int targetAngleTemp) {
    if(angleType == 0) {
        if(targetAngleTemp > angle0_absolute) {
            onNSZPressed();
            state_rotate = 0;
            targetAngle = targetAngleTemp;
            rotateTimer->start(100);
        } else if(targetAngleTemp < angle0_absolute) {
            onSSZPressed();
            state_rotate = 1;
            targetAngle = targetAngleTemp;
            rotateTimer->start(100);
        }
    } else if(angleType == 1) {
        if(targetAngleTemp > angle1_absolute) {
            ondropPressed();
            state_rotate = 2;
            targetAngle = targetAngleTemp;
            rotateTimer->start(100);
        } else if(targetAngleTemp < angle1_absolute) {
            onliftPressed();
            state_rotate = 3;
            targetAngle = targetAngleTemp;
            rotateTimer->start(100);
        }
    }
}

/**
 * @brief 更新角度0值
 * @param value 当前角度值
 */
void update_angle0(float value) {
    if(value == -1) return;
    if(angle0_first_flag) { angle0 = value; angle0_first_flag = false; return; }
    if(value > angle0 + 200) { angle0_round -= 1; saveSettings(); }
    else if(value < angle0 - 200) { angle0_round += 1; saveSettings(); }
    angle0 = value;
    angle0_absolute = angle0 + angle0_round*360;
    ui->labelCurrentLiftMotor->setText(QString::number(angle0_absolute));
}

/**
 * @brief 更新角度1值
 * @param value 当前角度值
 */
void update_angle1(float value) {
    if(value == -1) return;
    if(angle1_first_flag) { angle1 = value; angle1_first_flag = false; return; }
    if(value > angle1 + 200) { angle1_round -= 1; saveSettings(); }
    else if(value < angle1 - 200) { angle1_round += 1; saveSettings(); }
    angle1 = value;
    angle1_absolute = angle1 + angle1_round*360;
    ui->labelCurrentPitchMotor->setText(QString::number(angle1_absolute));
}

/**
 * @brief 更新左限位开关状态
 * @param value true表示触发，false表示未触发
 */
void update_leftLimit(bool value) {
    left_limit = value;
}

/**
 * @brief 更新右限位开关状态
 * @param value true表示触发，false表示未触发
 */
void update_rightLimit(bool value) {
    right_limit = value;
}

/**
 * @brief 更新Rx电机扭矩值
 * @param value 扭矩值
 */
void update_rx_motor_tq(float value) {
    rx_motor_tq = value;
}

/**
 * @brief 更新RX电机速度
 * @param value 当前电机速度值
 */
void update_rx_motor_speed(float value) {
    rx_motor_speed = value;
}

/**
 * @brief 更新顶部激光数据（井口检测）
 * @param value 激光测距值
 *
 * 如果检测到障碍或井口，可触发自动停止逻辑
 */
bool top_laser_auto_stop = false;
bool first_top_laser = true;
float last_top_data = 0;
void update_top_laser(float value) {
    // 当前代码逻辑被注释，可按需启用
}

/**
 * @brief 更新底部激光测距，用于平均测量
 * @param value 激光测距值
 */
int time_laser_scan = 0;
float sum = 0;
bool flag_laser_measure = false;
void update_down_laser(float value){
    downLaserMeter = value;
    if(flag_laser_measure){
        if(time_laser_scan < 5){
            sum += downLaserMeter;
            time_laser_scan += 1;
        } else {
            float measure = sum / 5.0;
            ui->laser_deep_measure->setText(QString::number(measure, 'f', 5));
            time_laser_scan = 0;
            sum = 0;
            flag_laser_measure = false;
        }
    }
}

/**
 * @brief 检查左限位开关
 * 如果触发，停止下降动作并显示警告
 */
void checkLeftLimit(){
    if(left_limit) {
        downButtonReleased();
        leftLimitTimer->stop();
        QMessageBox::warning(nullptr, nullptr, "warning", "到达高度上限");
    }
}

/**
 * @brief 检查右限位开关
 * 如果触发，停止下降动作并显示警告
 */
void checkRightLimit(){
    if(right_limit) {
        downButtonReleased();
        rightLimitTimer->stop();
        QMessageBox::warning(nullptr, nullptr, "warning", "到达高度下限");
    }
}

/**
 * @brief 云台复位操作，分状态机执行
 *
 * 状态：
 * 0 - 初始下降云台
 * 1 - 检查右限位
 * 2 - 根据angle0旋转
 * 3 - 根据angle1俯仰
 * 4 - 完成复位，停止定时器
 */
void resetAll(){
    switch(state) {
        case 0:
            state = 3;
            downButtonReleased();
            break;
        case 1:
            node.publishMessage("CL");
            state = 2;
            break;
        case 2:
            if(right_limit) downButtonReleased(), state = 3;
            break;
        case 3:
            if(angle0_absolute > angle0_zero + angle0_error) onSSZPressed();
            else if(angle0_absolute < angle0_zero - angle0_error) onNSZPressed();
            else cameraButtonReleased(), state = 4;
            break;
        case 4:
            if(angle1_absolute > angle1_zero + angle1_error) onliftPressed();
            else if(angle1_absolute < angle1_zero - angle1_error) ondropPressed();
            else cameraButtonReleased(), state = 0, resetTimer->stop();
            break;
    }
}

/**
 * @brief 云台旋转定时器槽函数
 * 根据state_rotate控制角度达到目标后停止旋转
 */
void rotateTimerSlot() {
    switch(state_rotate) {
        case 0:
            if(targetAngle <= angle0_absolute) cameraButtonReleased(), rotateTimer->stop(), state_rotate = -1;
            break;
        case 1:
            if(targetAngle >= angle0_absolute) cameraButtonReleased(), rotateTimer->stop(), state_rotate = -1;
            break;
        case 2:
            if(targetAngle <= angle1_absolute) cameraButtonReleased(), rotateTimer->stop(), state_rotate = -1;
            break;
        case 3:
            if(targetAngle >= angle1_absolute) cameraButtonReleased(), rotateTimer->stop(), state_rotate = -1;
            break;
        default:
            rotateTimer->stop();
    }
}

/**
 * @brief 更新里程计标签
 * @param speed 当前速度
 * @param position 当前位移
 */
double odom_data = 0;
void updateOdomLabels(double speed, double position) {
    ui->label_2->setText(QString::number(speed, 'f', 2));
    ui->label_14->setText(QString::number(position, 'f', 2));
    odom_data = position;
}

/**
 * @brief 急停按钮槽函数
 * 停止所有电机，停止MODBUS控制，并重置UI
 */
void on_Button_Stop_clicked() {
    QByteArray command2;
    command2.append('\x53'); command2.append('\x44'); command2.append('\x0D'); command2.append('\x0A');
    rosSender->sendCommand(command2);
    rosSender->sendCommand(command2);
    rosSender->sendCommand(command2);
    my_modbus->stopLine();
    ui->Button_FO->setText("向后");
    ui->Button_FO->setStyleSheet("color:#000000;");
}

/**
 * @brief 加速指令
 * 增加左右电机速度
 */
void Speed_Up() {
    left_speed += 20;
    right_speed += 20;
    QMessageBox::information(nullptr, nullptr, "info", QString("L_speed:%1, R_speed:%2").arg(left_speed).arg(right_speed));
    QByteArray command;
    command.append('\x41'); command.append('\x43'); command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
}

/**
 * @brief 减速指令
 * 减小左右电机速度
 */
void Speed_Down() {
    left_speed -= 20;
    right_speed -= 20;
    QMessageBox::information(nullptr, nullptr, "info", QString("L_speed:%1, R_speed:%2").arg(left_speed).arg(right_speed));
    QByteArray command;
    command.append('\x44'); command.append('\x43'); command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
}

/**
 * @brief 云台逆时针旋转（angle0增大）
 */
void onNSZPressed() {
    QByteArray command;
    command.append('\x5A'); command.append('\x55'); command.append('\x4F'); command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
}

/**
 * @brief 云台顺时针旋转（angle0减小）
 */
void onSSZPressed() {
    QByteArray command;
    command.append('\x59'); command.append('\x4F'); command.append('\x55'); command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
}

/**
 * @brief 云台俯视旋转（angle1增大）
 */
void ondropPressed() {
    QByteArray command;
    command.append('\x44'); command.append('\x4F'); command.append('\x57'); command.append('\x4E'); command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
}

/**
 * @brief 云台仰视旋转（angle1减小）
 */
void onliftPressed() {
    QByteArray command;
    command.append('\x55'); command.append('\x50'); command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
}

/**
 * @brief 云台升降向上
 */
void yuntai_liftPressed() {
    QByteArray command;
    command.append('\x43'); command.append('\x4C'); command.append('\x53'); command.append('\x53'); command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
    rosSender->sendCommand(command);
}

/**
 * @brief 云台升降向下
 */
void yuntai_downPressed() {
    QByteArray command;
    command.append('\x43'); command.append('\x53'); command.append('\x58'); command.append('\x4A'); command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
    rosSender->sendCommand(command);
}

/**
 * @brief 云台复位指令（停止角度电机）
 */
void yuntai_Reset() {
    QByteArray command;
    command.append('\x58'); command.append('\x5A'); command.append('\x53'); command.append('\x54'); command.append('\x4F'); command.append('\x50'); command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);

    QByteArray command1;
    command1.append('\x46'); command1.append('\x59'); command1.append('\x53'); command1.append('\x54'); command1.append('\x4F'); command1.append('\x50'); command1.append('\x0D'); command1.append('\x0A');
    rosSender->sendCommand(command1);
}

/**
 * @brief 停止云台、升降电机
 */
void downButtonReleased() {
    node.publishMessage("CS");
}

/**
 * @brief 停止云台旋转
 */
void cameraButtonReleased() {
    node.publishMessage("SS3");
    node.publishMessage("SS3");
    QThread::msleep(30);
    node.publishMessage("SS3");
}

/**
 * @brief 前置LED灯开启
 */
void FLED_On_Clicked() {
    QByteArray command;
    command.append('\x46'); command.append('\x4C'); command.append('\x45'); command.append('\x44'); command.append('\x4F'); command.append('\x4E'); command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
}

/**
 * @brief 前置LED灯关闭
 */
void FLED_Off_Clicked() {
    QByteArray command;
    command.append('\x46'); command.append('\x4C'); command.append('\x45'); command.append('\x44'); command.append('\x4F'); command.append('\x46'); command.append('\x46'); command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
}

/**
 * @brief 后置LED灯开启
 */
void WBLED_On_Clicked() {
    QByteArray command;
    command.append('\x42'); command.append('\x4C'); command.append('\x45'); command.append('\x44'); command.append('\x4F'); command.append('\x4E'); command.append('\x0D'); command.append('\x0A');
    rosSender->sendCommand(command);
}
/**
 * @brief 后置LED补光灯关闭
 * 
 * 生成BLEDOFF指令并通过rosSender发送到目标IP和端口。
 * 
 * @return void
 */
void BLED_Off_Clicked() {
    QByteArray command;
    command.append('\x42');
    command.append('\x4c');
    command.append('\x45');
    command.append('\x44');
    command.append('\x4F');
    command.append('\x46');
    command.append('\x46');
    command.append('\x0D');
    command.append('\x0A');

    rosSender->sendCommand(command);
}

/**
 * @brief 设置按钮，跳转到 SettingWidget 界面
 *
 * 如果实例不存在则创建，显示 SettingWidget 窗口。
 * 
 * @return void
 */
void on_pushButton_clicked()
{
    if (!settingWidgetInstance) {
        settingWidgetInstance = new SettingWidget(tempArgc,tempArgv,this);
    }
    settingWidgetInstance->setWindowFlag(Qt::Window);
    settingWidgetInstance->show();
}

/**
 * @brief 三维建图按钮，跳转到 JianTuWidget 界面
 *
 * 创建 JianTuWidget 实例并显示，同时发布 cloudmodel_pub 话题。
 * 
 * @return void
 */
void on_Button_JianTu_clicked()
{
    if (!jiantuWidgetInstance){
        jiantuWidgetInstance = new JianTuWidget(tempArgc,tempArgv,nullptr);
        my_rviz=new Qrviz(jiantuWidgetInstance->VerticalLayout);
    }
    jiantuWidgetInstance->showNormal();
    jiantuWidgetInstance->update();

    std_msgs::Bool msg;
    msg.data = true;
    node.publish_cloudmodel_pub(msg);
}

/**
 * @brief 设置自主导航距离并开始导航，同时启动报告生成
 *
 * 读取 lineEdit_Distance 输入框内容，如果为空或非整数则弹出警告。
 * 构造 finalgoal 消息发布自动导航指令，同时更新 label_17 提示报告生成状态。
 * 
 * @return void
 */
void on_Button_distance_clicked()
{
    QString distanceText = ui->lineEdit_Distance->text();
    if (distanceText.isEmpty()) {
        QMessageBox::warning(this, "输入错误", "请输入自主导航距离。");
        return;
    }

    bool ok;
    int distance = distanceText.toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "输入错误", "自主导航距离必须是整数。");
        return;
    }

    geometry_msgs::PoseStamped finalgoal;
    finalgoal.pose.position.x = static_cast<double>(distance);
    node.publishAutonomousNavigation(finalgoal);

    ui->label_17->setText("正在记录检测报告，需要手动关闭报告生成");

    std_msgs::Bool msg;
    msg.data = true;
    node.publish_report_generate(msg);

    start_auto = true;
}

/**
 * @brief 停止报告生成
 *
 * 发布 whether_report_generate 消息为 false，同时更新界面提示。
 * 
 * @return void
 */
void on_Button_report_finish_clicked()
{
    std_msgs::Bool msg;
    msg.data = false;
    node.publish_report_generate(msg);
    ui->label_17->setText("报告已生成");
}

/**
 * @brief 计米器归零
 *
 * 发布计米器归零消息。
 * 
 * @return void
 */
void on_Button_ResetMeter_clicked()
{
    node.publishResetMeter();
}

/**
 * @brief 接收 auto_navigation 状态更新
 *
 * @param msg 当前自动导航状态
 * @return void
 */
void update_auto_navigation(bool msg)
{
    // 逻辑暂时注释
}

/**
 * @brief 打开参数界面
 *
 * 根据当前GPS经纬度显示创建 Param 窗口实例并显示。
 * 
 * @return void
 */
void on_checkParamButton_clicked()
{
    Param *paramWidegt = new Param(tempArgc, tempArgv,
                                   ui->labelLon->text(), ui->labelLat->text(),nullptr);
    paramWidegt->show();
}

/**
 * @brief 停止复位操作
 *
 * 发送 SJSTOP 指令停止复位，并停止 resetTimer，同时释放相关按键状态。
 * 
 * @return void
 */
void on_stopResetButton_clicked()
{
    QByteArray command;
    command.append('\x53');
    command.append('\x4A');
    command.append('\x53');
    command.append('\x54');
    command.append('\x4F');
    command.append('\x50');
    command.append('\x0D');
    command.append('\x0A');

    rosSender->sendCommand(command);
    resetTimer->stop();
    downButtonReleased();
    cameraButtonReleased();
    state = 0;
}

/**
 * @brief 旋转360度按钮槽
 *
 * 调用 rotateToTargetAngle 执行旋转。
 * 
 * @return void
 */
void on_rotate360Button_clicked()
{
    rotateToTargetAngle(0, angle0_zero - 355);
}

/**
 * @brief 旋转90度按钮槽
 *
 * 调用 rotateToTargetAngle 执行旋转。
 * 
 * @return void
 */
void on_rotate90Button_clicked()
{
    rotateToTargetAngle(1, angle1_zero + 65);
}

/**
 * @brief 向后自动行走按钮槽
 *
 * 调用 Modbus 的 autoLineBack 函数实现向后自动移动。
 *
 * @return void
 */
void on_rxpushButton_2_clicked()
{
    my_modbus->autoLineBack(29);
}

/**
 * @brief 停止行走按钮槽
 *
 * 调用 Modbus 的 stopLine 函数停止当前运动。
 *
 * @return void
 */
void on_pushButton_8_clicked()
{
    my_modbus->stopLine();
}

/**
 * @brief ROS FO 信号触发处理槽
 *
 * 当接收到 ROS FO 消息时触发，发送 FO 指令到目标IP和端口。
 *
 * @return void
 */
void slotFOTriggered()
{
    qDebug() << "Received FO from ROS, triggering button logic...";
    QByteArray command;
    command.append('\x46');
    command.append('\x4F');
    command.append('\x0D');
    command.append('\x0A');
    rosSender->sendCommand(command);
}

/**
 * @brief ROS BA 信号触发处理槽
 *
 * 当接收到 ROS BA 消息时触发，发送 BA 指令到目标IP和端口。
 *
 * @return void
 */
void slotBATriggered()
{
    qDebug() << "Received BA from ROS, triggering button logic...";
    QByteArray command1;
    command1.append('\x42');
    command1.append('\x41');
    command1.append('\x0D');
    command1.append('\x0A');
    rosSender->sendCommand(command1);
}

/**
 * @brief ROS ST 信号触发处理槽
 *
 * 当接收到 ROS ST 消息时触发，发送 SD 指令到目标IP和端口。
 *
 * @return void
 */
void slotSTTriggered()
{
    qDebug() << "Received ST from ROS, triggering button logic...";
    QByteArray command2;
    command2.append('\x53');
    command2.append('\x44');
    command2.append('\x0D');
    command2.append('\x0A');
    rosSender->sendCommand(command2);
}

/**
 * @brief 前进按钮槽
 *
 * 根据当前按钮状态发送 FO 指令启动前进，或者发送 SD 指令停止前进。
 * 仅在其它方向按钮未被按下时有效。
 *
 * @return void
 */
void on_Button_FO_clicked()
{
    if(BUTTONBA==0&&BUTTONLE==0&&BUTTONRI==0){
        BUTTONFO=1;

        if(ui->Button_FO->text() == "向前") {
            QThread::msleep(500);
            QByteArray command;
            command.append('\x46');
            command.append('\x4F');
            command.append('\x0D');
            command.append('\x0A');
            rosSender->sendCommand(command);

            ui->Button_FO->setText("停止");
            ui->Button_FO->setStyleSheet("color:#FF0000;");
        } else {
            QByteArray command1;
            command1.append('\x53');
            command1.append('\x44');
            command1.append('\x0D');
            command1.append('\x0A');
            rosSender->sendCommand(command1);
            rosSender->sendCommand(command1);
            rosSender->sendCommand(command1);
            my_modbus->stopLine();

            ui->Button_FO->setText("向前");
            ui->Button_FO->setStyleSheet("color:#000000;");
            BUTTONFO=0;
        }
    }
}

/**
 * @brief 后退按钮槽
 *
 * 根据当前按钮状态发送 BA 指令启动后退，或者发送 SD 指令停止后退。
 * 仅在其它方向按钮未被按下时有效。
 *
 * @return void
 */
void on_Button_BA_clicked()
{
    if(BUTTONFO==0&&BUTTONLE==0&&BUTTONRI==0){
        BUTTONBA=1;
        if(ui->Button_BA->text() == "向后") {
            QThread::msleep(500);
            QByteArray command1;
            command1.append('\x42');
            command1.append('\x41');
            command1.append('\x0D');
            command1.append('\x0A');
            rosSender->sendCommand(command1);

            ui->Button_BA->setText("停止");
            ui->Button_BA->setStyleSheet("color:#FF0000;");
        } else {
            QByteArray command2;
            command2.append('\x53');
            command2.append('\x44');
            command2.append('\x0D');
            command2.append('\x0A');
            rosSender->sendCommand(command2);

            ui->Button_BA->setText("向后");
            ui->Button_BA->setStyleSheet("color:#000000;");
            BUTTONBA=0;
        }
    }
}

/**
 * @brief 左转按钮槽
 *
 * 根据当前按钮状态发送 LE 指令启动左转，或者发送 SD 指令停止左转。
 * 仅在其它方向按钮未被按下时有效。
 *
 * @return void
 */
void on_Button_LF_clicked()
{
    if(BUTTONFO==0&&BUTTONBA==0&&BUTTONRI==0){
        BUTTONLE=1;
        if(ui->Button_LF->text() == "向左") {
            int speed = 31;
            QThread::msleep(500);
            QByteArray command3;
            command3.append('\x4C');
            command3.append('\x45');
            command3.append('\x0D');
            command3.append('\x0A');
            rosSender->sendCommand(command3);

            ui->Button_LF->setText("停止");
            ui->Button_LF->setStyleSheet("color:#FF0000;");
        } else {
            QByteArray command2;
            command2.append('\x53');
            command2.append('\x44');
            command2.append('\x0D');
            command2.append('\x0A');
            rosSender->sendCommand(command2);
            rosSender->sendCommand(command2);
            rosSender->sendCommand(command2);

            ui->Button_LF->setText("向左");
            ui->Button_LF->setStyleSheet("color:#000000;");
            BUTTONLE=0;
        }
    }
}

/**
 * @brief 右转按钮槽
 *
 * 根据当前按钮状态发送 RI 指令启动右转，或者发送 SD 指令停止右转。
 * 仅在其它方向按钮未被按下时有效。
 *
 * @return void
 */
void on_Button_RT_clicked()
{
    if(BUTTONFO==0&&BUTTONBA==0&&BUTTONLE==0){
        BUTTONRI=1;
        if(ui->Button_RT->text() == "向右") {
            int speed = 31;
            QThread::msleep(500);
            QByteArray command2;
            command2.append('\x52');
            command2.append('\x49');
            command2.append('\x0D');
            command2.append('\x0A');
            rosSender->sendCommand(command2);

            ui->Button_RT->setText("停止");
            ui->Button_RT->setStyleSheet("color:#FF0000;");
        } else {
            QByteArray command2;
            command2.append('\x53');
            command2.append('\x44');
            command2.append('\x0D');
            command2.append('\x0A');
            rosSender->sendCommand(command2);
            rosSender->sendCommand(command2);
            rosSender->sendCommand(command2);

            ui->Button_RT->setText("向右");
            ui->Button_RT->setStyleSheet("color:#000000;");
            BUTTONRI=0;
        }
    }
}

/**
 * @brief 左右电机手动速度设置按钮槽
 *
 * 读取 lineEdit 左右速度值，生成 L/R 指令并通过 ROS 发布。
 *
 * @return void
 */
void on_pushButton_2_clicked()
{
    int left_s = ui->lineEdit_left_s->text().toInt();
    int right_s = ui->lineEdit_right_s->text().toInt();
    QString str = QString("L%1R%2").arg(left_s).arg(right_s);
    qDebug() << QString::fromStdString(str.toStdString());
    node.publishMessage(str.toStdString());
}
/**
 * @brief 接收并显示经纬度：经度
 * 
 * 将 ROS 或串口传来的经度值显示在 UI labelLon 上。
 *
 * @param lon 双精度经度数值
 * @return void
 */
void onLonRecived(double lon) {
    ui->labelLon->setText(QString::number(lon, 'f', 5));
}

/**
 * @brief 接收并显示经纬度：纬度
 *
 * 将 ROS 或串口传来的纬度值显示在 UI labelLat 上。
 *
 * @param lat 双精度纬度数值
 * @return void
 */
void onLatRecived(double lat) {
    ui->labelLat->setText(QString::number(lat, 'f', 5));
}

/**
 * @brief 截图按钮槽函数
 *
 * 截取整个主窗口，弹出对话框选择保存路径，并保存 PNG/JPG。
 * 保存成功或失败会弹出提示框。
 *
 * @return void
 */
void on_snapshot_button_clicked()
{
    qDebug() << "==========================snapshot";
    QPixmap screenshot = this->grab();

    if(!screenshot.isNull()) {
        QString fileName = QFileDialog::getSaveFileName(
            this,
            "保存截图",
            QDir::homePath(),
            "图片文件 (*.png *.jpg *.jpeg)"
        );

        if(!fileName.isEmpty()) {
            if(screenshot.save(fileName)) {
                QMessageBox::information(this, "成功", "截图保存成功");
            } else {
                QMessageBox::warning(this, "错误", "保存文件失败");
            }
        }
    } else {
        QMessageBox::warning(this, "错误", "截图获取失败");
    }
}

/**
 * @brief 打开电机速度测试窗口
 *
 * 创建 MotorSpeedTest 实例并显示为独立窗口，
 * 若窗口已存在则直接显示。
 *
 * @return void
 */
void on_motor_speed_test_button_clicked()
{
    if (!motorSpeedTestWidgetInstance) {
        motorSpeedTestWidgetInstance = new MotorSpeedTest(tempArgc, tempArgv, nullptr);
    }
    motorSpeedTestWidgetInstance->setWindowFlag(Qt::Window);
    motorSpeedTestWidgetInstance->show();
}

/**
 * @brief 启动机器人下位机程序
 *
 * 调用 run_robot.sh 脚本启动下位机 ROS 节点。
 * 成功启动后弹出提示框。
 *
 * @return void
 */
void on_start_robot_button_clicked()
{
    QString scriptPath = "/home/ralab/Pipelineqt/Pipeline/mySrc/run_robot.sh";
    remoteProcess->start("sh", QStringList() << scriptPath);
    QThread::sleep(3);
    QMessageBox::information(this, "成功", "下位机开启成功");
}

/**
 * @brief 远程关闭机器人下位机 ROS
 *
 * 使用 sshpass + ssh 执行远程 kill 命令，
 * 若 PID 不存在则输出提示。
 *
 * @return void
 */
void on_quit_robot_button_clicked()
{
    QProcess process;
    QString password = "lee";
    QString remoteUser = "lee";
    QString remoteIp = "192.168.8.7";

    QStringList args;
    args << "-p" << password
         << "ssh"
         << QString("%1@%2").arg(remoteUser).arg(remoteIp)
         << "kill -TERM $(cat /tmp/roslaunch.pid) 2>/dev/null || echo '进程不存在或PID文件已丢失'";

    process.start("sshpass", args);

    if (!process.waitForFinished(5000)) {
        qDebug() << "执行超时或发生错误:" << process.errorString();
        QMessageBox::warning(this, "成功", "关闭下位机失败");
        return;
    }

    qDebug() << "退出码:" << process.exitCode();
    qDebug() << "标准输出:" << process.readAllStandardOutput();
    qDebug() << "错误输出:" << process.readAllStandardError();
}

/**
 * @brief 录制视频按钮槽函数
 *
 * 第一次点击：开始录制视频，自动命名保存到 ~/Videos/
 * 再次点击：停止录制并恢复按钮样式
 *
 * UI 使用 reset_robot_button 显示录制状态（红色/蓝色）
 *
 * @return void
 */
void on_start_record_button_clicked()
{
    if (!isRecording) {
        QString filename = QDir::homePath()+"/Videos/"+QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + ".mp4";

        if (m_recorder->startRecording(filename, m_currentFrame.width(),
                                       m_currentFrame.height(), 25)) {

            ui->reset_robot_button->setText("正在录制");
            ui->reset_robot_button->setStyleSheet(
                "QPushButton { background-color: #FF0000; color: white; border-radius: 5px; padding:5px; }"
                "QPushButton:hover { background-color: #CC0000; }"
                "QPushButton:pressed { background-color: #AA0000; }"
            );
            isRecording = true;
        }
    } else {
        m_recorder->stopRecording();

        ui->reset_robot_button->setText("开始录制");
        ui->reset_robot_button->setStyleSheet(
            "QPushButton { background-color: #007ACC; color: white; border-radius: 5px; padding:5px; }"
            "QPushButton:hover { background-color: #005B99; }"
            "QPushButton:pressed { background-color: #004D80; }"
        );
        isRecording = false;
    }
}

/**
 * @brief 视频录制完成后的回调函数
 *
 * 外部 VideoRecorder 调用此槽，
 * 提示用户录制的视频文件保存位置。
 *
 * @param filename 保存的视频文件路径
 * @return void
 */
void onRecordingFinished(const QString &filename)
{
    QMessageBox::information(this, "录制完成",
                             "视频已保存为: " + filename);
}

/**
 * @brief 更新 UI 显示的视频图像
 *
 * 将输入 QImage 缩放并显示到 UI label_35 中，
 * 保持比例并使用平滑缩放。
 *
 * @param img 显示的图像帧
 * @return void
 */
void updateImage(const QImage &img)
{
    if(img.isNull()) return;

    ui->label_35->setPixmap(QPixmap::fromImage(img).scaled(
        ui->label_35->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation
    ));
}

/**
 * @brief 经度回调处理槽函数
 *
 * 接收到经度数值后，将其格式化为字符串并显示到 UI 标签 labelLon 上。
 *
 * @param lon 接收到的经度值（单位：度）
 */
void onLonRecived(double lon) {
   ui->labelLon->setText(QString::number(lon, 'f', 5));
}

/**
 * @brief 纬度回调处理槽函数
 *
 * 接收到纬度数值后，将其格式化为字符串并显示到 UI 标签 labelLat 上。
 *
 * @param lat 接收到的纬度值（单位：度）
 */
void onLatRecived(double lat) {
  ui->labelLat->setText(QString::number(lat, 'f', 5));
}

/**
 * @brief 截图按钮点击事件槽函数
 *
 * 执行步骤：
 * 1. 对当前窗口进行截图
 * 2. 用户选择保存路径
 * 3. 保存为 PNG / JPG 格式
 * 4. 成功或失败均会弹出提示框
 */
void on_snapshot_button_clicked()
{
    qDebug() << "==========================snapshot";
    QPixmap screenshot = this->grab();

    if(!screenshot.isNull()) {
        QString fileName = QFileDialog::getSaveFileName(
            this,
            "保存截图",
            QDir::homePath(),
            "图片文件 (*.png *.jpg *.jpeg)"
        );

        if(!fileName.isEmpty()) {
            if(screenshot.save(fileName)) {
                QMessageBox::information(this, "成功", "截图保存成功");
            } else {
                QMessageBox::warning(this, "错误", "保存文件失败");
            }
        }
    } else {
        QMessageBox::warning(this, "错误", "截图获取失败");
    }
}

/**
 * @brief 打开电机速度测试窗口
 *
 * 若 motorSpeedTestWidgetInstance 尚未创建，则动态创建并显示。
 */
void on_motor_speed_test_button_clicked()
{
    if (!motorSpeedTestWidgetInstance) {
        motorSpeedTestWidgetInstance = new MotorSpeedTest(tempArgc, tempArgv, nullptr);
    }
    motorSpeedTestWidgetInstance->setWindowFlag(Qt::Window);
    motorSpeedTestWidgetInstance->show();
}

/**
 * @brief 启动远程机器人（运行 run_robot.sh）
 *
 * 调用远程脚本用于启动下位机。
 */
void on_start_robot_button_clicked()
{
    QString scriptPath = "/home/ralab/Pipelineqt/Pipeline/mySrc/run_robot.sh";
    remoteProcess->start("sh", QStringList() << scriptPath);
    QThread::sleep(3);
    QMessageBox::information(this, "成功", "下位机开启成功");
}

/**
 * @brief 停止远程机器人（关闭 ROS launch）
 *
 * 通过 sshpass 执行远程 kill 命令关闭下位机 roslaunch 进程。
 */
void on_quit_robot_button_clicked()
{
    QProcess process;
    QString password = "lee";
    QString remoteUser = "lee";
    QString remoteIp = "192.168.8.7";

    QStringList args;
    args << "-p" << password
         << "ssh"
         << QString("%1@%2").arg(remoteUser).arg(remoteIp)
         << "kill -TERM $(cat /tmp/roslaunch.pid) 2>/dev/null || echo '进程不存在或PID文件已丢失'";

    process.start("sshpass", args);

    if (!process.waitForFinished(5000)) {
        qDebug() << "执行超时:" << process.errorString();
        QMessageBox::warning(this, "错误", "关闭下位机失败");
        return;
    }

    qDebug() << "退出码:" << process.exitCode();
    qDebug() << "标准输出:" << process.readAllStandardOutput();
    qDebug() << "错误输出:" << process.readAllStandardError();
}

/**
 * @brief 点击按钮开始或停止录制视频
 *
 * 使用自定义 Recorder 类录制当前视频帧。
 * 按钮会动态切换状态、颜色与文字。
 */
void on_start_record_button_clicked()
{
    if (!isRecording) {
        QString filename = QDir::homePath()+"/Videos/"+QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + ".mp4";

        if (m_recorder->startRecording(filename, m_currentFrame.width(),
                                      m_currentFrame.height(), 25)) {

            ui->reset_robot_button->setText("正在录制");
            ui->reset_robot_button->setStyleSheet(
                "QPushButton { background-color: #FF0000; color: white; }"
            );
            isRecording = true;
        }
    } else {

        m_recorder->stopRecording();

        ui->reset_robot_button->setText("开始录制");
        ui->reset_robot_button->setStyleSheet(
            "QPushButton { background-color: #007ACC; color: white; }"
        );
        isRecording = false;
    }
}

/**
 * @brief 视频录制完成后的回调槽函数
 *
 * @param filename 已保存的视频文件路径
 */
void onRecordingFinished(const QString &filename)
{
    QMessageBox::information(this, "录制完成",
                           "视频已保存为: " + filename);
}

/**
 * @brief 将图像显示到 UI 的 label_35 中
 *
 * @param img 传入的图像
 */
void updateImage(const QImage &img)
{
    if(img.isNull()) return;

    ui->label_35->setPixmap(QPixmap::fromImage(img).scaled(
        ui->label_35->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation
    ));
}

/**
 * @brief 远程重启机器人（关机）
 *
 * 发送 sudo shutdown -h now
 * 若 SSH 中断，也认为命令已发送。
 */
void on_reset_robot_button_2_clicked()
{
    QProcess process;
    QString password = "lee";
    QString remoteUser = "lee";
    QString remoteIp = "192.168.8.7";

    QStringList args;
    args << "-p" << password
         << "ssh"
         << QString("%1@%2").arg(remoteUser).arg(remoteIp)
         << "echo '" + password + "' | sudo -S shutdown -h now";

    process.start("sshpass", args);

    if (!process.waitForFinished(30000)) {
        qDebug() << "连接断开，可能已触发重启:" << process.errorString();
        QMessageBox::information(this, "提示", "已发送重启指令，请等待主机重启");
    }
    else {
        QMessageBox::information(this, "成功", "远程主机已关机，需要断电才能重启");
    }
}

/**
 * @brief 激光井深测量按钮
 *
 * 设置激光测距触发标志位
 */
void on_laser_deep_measure_button_clicked()
{
    flag_laser_measure = true;
}

/**
 * @brief 线缆井深度测量按钮
 *
 * 点击一次 → 开始测量  
 * 再点击一次 → 结束测量并显示结果
 */
void on_line_deep_measure_button_clicked()
{
    if (ui->line_deep_measure_button->text() == "线缆井深测量") {

        ui->line_deep_measure_button->setText("结束测量");
        ui->line_deep_measure_button->setStyleSheet("color:#FF0000;");
        start_measure_dis = odom_data;

    } else {

        ui->line_deep_measure_button->setText("线缆井深测量");
        ui->line_deep_measure_button->setStyleSheet("color:#000000;");
        end_measure_dis = odom_data;

        ui->line_deep_measure->setText(QString::number(end_measure_dis - start_measure_dis, 'f', 5));

        start_measure_dis = 0;
        end_measure_dis = 0;
    }
}

/**
 * @brief 延迟数据显示槽
 *
 * @param latency 平均网络延迟（毫秒）
 */
void onLatencyUpdated(double latency)
{
    ui->label_34->setText(QString("延迟: %1 ms").arg(latency, 0, 'f', 1));
    qDebug() << "Average latency:" << latency << "ms";
}

/**
 * @brief 切换 RTSP 摄像头按钮
 *
 * 停止解码器 → 修改 URL → 重启解码器
 */
void on_pushButton_3_clicked()
{
    decoder_->requestInterruption();
    decoder_->wait();

    decoder_->setUrl(flag ?
                     "rtsp://admin:123456@192.168.8.10/Streaming/Channels/2" :
                     "rtsp://admin:123456@192.168.8.9/Streaming/Channels/2");

    decoder_->start();
}

/**
 * @brief 保存当前的三轴角度（用于校准）
 */
void on_pushButton_5_clicked()
{
    roll_ave1 = roll_ave;
    pitch_ave1 = pitch_ave;
    yaw_ave1 = yaw_ave;
}


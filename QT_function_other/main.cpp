/**
 * @file main.cpp
 * @brief 程序入口文件，初始化 Qt 应用、启动 ROS 脚本、创建主窗口 Widget 并执行应用循环
 *
 * 功能说明：
 * - 初始化 QApplication
 * - 启动 ROS 节点和相关 shell 脚本
 * - 设置 QSS 样式表
 * - 创建 Widget 主窗口并显示
 * - 注册程序退出时终止 ROS 脚本进程的回调
 */

#include <QApplication>
#include <ros/ros.h>
#include "qrviz.h"
#include <QProcess>
#include <QtGui>
#include <QThread>
#include "qnode.h"
#include "widget.h"
#include "sql.h"
#include "form.h"
#include "settingwidget.h"
#include <catkin_ws/devel/include/pipeline/HeaderInfo.h>

/**
 * @brief 程序主入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 返回程序执行状态，0 表示正常退出
 *
 * 主函数主要功能：
 * - 创建 QApplication 对象
 * - 启动 ROS 脚本 run_topnx.sh，并输出日志到控制台
 * - 注册应用退出时终止 ROS 脚本进程的回调
 * - 初始化 ROS 节点
 * - 创建 Widget 主窗口对象
 * - 设置应用样式表（QSS）
 * - 显示主窗口并进入事件循环
 */
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    // 启动 ROS 脚本
    QProcess process;
    QString scriptPath = "/home/ralab/Pipelineqt/Pipeline/mySrc/run_topnx.sh";

    // 连接标准输出和错误输出
    QObject::connect(&process, &QProcess::readyReadStandardOutput, [&]() {
        qDebug() << "脚本输出：" << process.readAllStandardOutput();
    });
    QObject::connect(&process, &QProcess::readyReadStandardError, [&]() {
        qDebug() << "脚本错误：" << process.readAllStandardError();
    });

    process.start("sh", QStringList() << scriptPath);

    // 程序退出时终止 ROS 脚本进程
    QObject::connect(&app, &QCoreApplication::aboutToQuit, [&]() {
        QFile file("/tmp/roslaunch.pid");
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            qWarning() << "无法打开PID文件";
            return;
        }

        QTextStream in(&file);
        QString pidStr = in.readLine().trimmed();
        file.close();

        bool ok;
        int pid = pidStr.toInt(&ok);
        if (!ok) {
            qWarning() << "PID无效";
            return;
        }

        QProcess killProcess;
        QStringList args;
        args << QString::number(pid);
        killProcess.start("kill", args);

        if (!killProcess.waitForFinished(5000)) {
            qWarning() << "终止进程超时或出错";
            return;
        }

        if (killProcess.exitCode() != 0) {
            qWarning() << "终止进程失败，错误码:" << killProcess.exitCode();
            args.prepend("-9");
            killProcess.start("kill", args);
            killProcess.waitForFinished();
        } else {
            qDebug() << "进程终止成功";
        }
    });

    qDebug() << "wait to start...................";
    QThread::sleep(6);

    // 初始化 ROS 节点
    ros::init(argc, argv,"test_rviz");

    // 创建 Widget 主窗口
    Widget *monitorWindow = new Widget(argc, argv);

    // 加载 QSS 样式表
    QFile file(":/mySrc/c.css");
    if(file.open(QFile::ReadOnly)) {
        QString styleSheet = QLatin1String(file.readAll());
        app.setStyleSheet(styleSheet);
        file.close();
    } else {
        qDebug() << "Cannot open QSS file:" << file.errorString();
    }

    monitorWindow->show();

    return app.exec();
}

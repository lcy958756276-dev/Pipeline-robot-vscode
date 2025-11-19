/**
 * @file modbus.cpp
 * @brief Modbus 通信类实现，用于通过串口控制下位机电机
 *
 * 功能：
 * - 初始化串口参数
 * - 发送 Modbus 指令（写单个寄存器、控制电机前进/后退/停止）
 * - 计算 CRC16 校验
 * - 提供自动前进、自动后退、停止和禁用线路功能
 */

#include "modbus.h"

/**
 * @brief 构造函数，初始化串口参数
 */
modbus::modbus() {
    serialPort.setPortName("/dev/ttyTHS0");
    serialPort.setBaudRate(QSerialPort::Baud19200);
    serialPort.setDataBits(QSerialPort::Data8);
    serialPort.setParity(QSerialPort::NoParity);
    serialPort.setStopBits(QSerialPort::OneStop);
    serialPort.setFlowControl(QSerialPort::NoFlowControl);
}

/**
 * @brief 初始化 Modbus 串口并打开
 */
void mod_init()
{
    serialPort.setPortName("/dev/ttyTHS0");
    serialPort.setBaudRate(QSerialPort::Baud19200);
    serialPort.setDataBits(QSerialPort::Data8);
    serialPort.setParity(QSerialPort::NoParity);
    serialPort.setStopBits(QSerialPort::OneStop);
    serialPort.setFlowControl(QSerialPort::NoFlowControl);
    if(serialPort.open(QIODevice::ReadWrite))
    {
        qDebug() << "串口打开成功";
    }
    else
    {
        qDebug() << "串口打开异常:" << serialPort.errorString();
    }
}

/**
 * @brief 析构函数，关闭串口
 */
modbus::~modbus()
{
    if(serialPort.isOpen()){
        serialPort.close();
    }
}

/**
 * @brief 发送 Modbus 指令帧
 * @param frame 指令帧数据
 * @param length 指令长度
 */
void Modbus_Send(uint8_t *frame, uint16_t length)
{
    if(serialPort.isOpen() && serialPort.isWritable()){
        for (int i = 0; i < length; i++) {
            char byte = static_cast<char>(frame[i]);
            serialPort.write(&byte, 1);
            serialPort.flush();
        }
    } else {
        qDebug() << "Modbus 发送错误";
    }
}

/**
 * @brief 计算 Modbus CRC16 校验
 * @param data 数据缓冲区
 * @param length 数据长度
 * @return uint16_t CRC16 校验值
 */
uint16_t Modbus_CRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < length; pos++) {
        crc ^= static_cast<uint16_t>(data[pos]);
        for (int i = 0; i < 8; i++) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 发送写单个寄存器的 Modbus 指令（功能码 0x06）
 */
void Modbus_Poll_6()
{
    uint8_t request[8];
    uint16_t crc;
    request[0] = 0x01;
    request[1] = 0x06;
    request[2] = TX_data[0];
    request[3] = TX_data[1];
    request[4] = TX_data[2];
    request[5] = TX_data[3];

    crc = Modbus_CRC16(request, 6);
    request[6] = crc & 0xFF;
    request[7] = (crc >> 8) & 0xFF;

    Modbus_Send(request, 8);
}

/**
 * @brief 自动后退指定扭矩
 * @param speed 后退扭矩
 */
void autoLineBack(int speed) {
    int speed_temp = -speed;
    qDebug() << "启动后退";
    TX_data[0]=0x05; TX_data[1]=0x14; TX_data[2]=0x00; TX_data[3]=0x10;
    Modbus_Poll_6();
    QThread::msleep(500);
    TX_data[0]=0x03; TX_data[1]=0x21; TX_data[2]=(speed_temp >> 8) & 0xFF; TX_data[3]=speed_temp & 0xFF;
    Modbus_Poll_6();
}

/**
 * @brief 自动前进指定扭矩
 * @param speed 前进扭矩
 */
void autoLineForward(int speed) {
    int speed_temp = speed;
    qDebug() << "启动前进";
    TX_data[0]=0x05; TX_data[1]=0x14; TX_data[2]=0x00; TX_data[3]=0x10;
    Modbus_Poll_6();
    QThread::msleep(500);
    TX_data[0]=0x03; TX_data[1]=0x21; TX_data[2]=(speed_temp >> 8) & 0xFF; TX_data[3]=speed_temp & 0xFF;
    Modbus_Poll_6();
}

/**
 * @brief 停止电机
 */
void stopLine() {
    qDebug() << "停止电机";
    TX_data[0]=0x03; TX_data[1]=0x21; TX_data[2]=0x00; TX_data[3]=0x00;
    Modbus_Poll_6();
    QThread::msleep(500);
    TX_data[0]=0x05; TX_data[1]=0x14; TX_data[2]=0x00; TX_data[3]=0x00;
    Modbus_Poll_6();
}

/**
 * @brief 禁用电机线路
 */
void disableLine() {
    qDebug() << "禁用电机";
    TX_data[0]=0x05; TX_data[1]=0x14; TX_data[2]=0x00; TX_data[3]=0x00;
    Modbus_Poll_6();
}

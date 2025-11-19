/**
 * @file torque.cpp
 * @brief 绕线盘与丝杆系统扭矩计算类
 *
 * 功能：
 * - 根据绳索长度计算绕线盘半径
 * - 计算绕线盘与丝杆的总质量
 * - 计算推力、摩擦力
 * - 根据几何关系计算总转矩
 * - 输出百分比形式的扭矩
 */

#include <torque.h>
#include <QDebug>
#include <qmath.h>
#include <math.h>

#define PI 3.14159265358979323846

// 系统几何尺寸
#define l1 0.18  ///< 绕线盘与编码器中心的水平距离
#define L1 0.2   ///< 旋转轮中心与地面的垂直距离
#define l3 0.2   ///< 编码器中心与滚轮中心水平距离
#define d1 0.02  ///< 绕线盘与编码器中心的垂直距离
#define d2 0.21  ///< 绕线盘中心与地面的距离
#define d3 0.1   ///< 编码器中心与滚轮上侧的垂直距离
#define a1 5     ///< 绳子绕满一圈所需长度
#define d4 0.17  ///< 车身上电缆连接的垂直距离
#define LLA 100  ///< 绳子的总长度
#define A 215/8.5///< 绳子绕满一圈所需圈数

// 质量和力学参数
#define M1 3     ///< 绕线盘质量
#define M3 1     ///< 丝杠上方矫正器质量
#define g 9.81   ///< 重力加速度
#define u1 0.1   ///< 绕线盘摩擦力系数
#define u2 0.2   ///< 绳子与地面摩擦系数
#define u3 0.2   ///< 丝杠摩擦系数
#define R_1 0.035///< 绕线盘内径，用于自身重量力矩计算
#define LB 2     ///< 线缆悬空长度
#define Fa_1 5   ///< 推力补偿
#define P 0.02   ///< 丝杠螺距，单位 m

/**
 * @brief 构造函数，初始化扭矩计算参数
 */
torque::torque() {
    R = ((LLA - La) / A) * (0.0085 / 2) + 0.065;        // 初始绕线盘半径
    M2 = M1 + (LLA - La) * 0.12;                        // 绕线盘与绳子的总质量
    Fa = M3 * g * u3 + Fa_1;                            // 丝杆推力
    Ta = (Fa * P) / (2 * PI);                           // 推力与扭矩关系
    Tc = (Ta / Zi) * q;                                 // 调整系数后的扭矩
}

/**
 * @brief 根据绳索长度 La_1 计算总扭矩 TG
 * @param La_1 当前绳索长度
 *
 * 计算流程：
 * 1. 根据几何关系计算绕线盘角度和坐标
 * 2. 计算绳索各段长度与角度
 * 3. 计算摩擦力产生的扭矩
 * 4. 合成各段力矩得到总扭矩 TG
 * 5. 输出百分比形式 TG_1
 */
void torque::TG_Measure(double La_1) {
    La = La_1;

    // 绕线盘几何角度计算
    double angle_1 = atan(d1 / l1) * 180 / PI;
    l2 = sqrt(d1*d1 + l1*l1);
    double angle_2 = acos(R / l2) * 180 / PI;
    double angle_3 = 90 - (angle_2 + angle_1);
    double angle_rad_3 = angle_3 / 180.0 * PI;

    // 坐标计算
    double Xa = R * sin(angle_rad_3);
    double Ya = R * cos(angle_rad_3) + d2;
    double Xb = l1;
    double Yb = d1 + d2;
    double Xc = l1 + l3;
    double Yc = d1 + d2 + d3;
    double Xd = l1 + l3 + La;
    double Yd = 0.17;

    // 绳索长度与拉伸计算
    l4 = Xd - l1 - l3;
    L4 = sqrt(pow((d2+d1+d3-d4),2) + pow(l4,2));

    // 更新绕线盘半径
    if (L4 < previous_D) {
        R += 0.0085 / 2;
        DL = 2 * PI * R;
        DD = DL * A;
        previous_D = L4 - DD;
    }

    // 绕线盘与绳子重量的摩擦扭矩
    double W = M2 * g;
    double Ff = W * u1;
    double Tf = Ff * R;

    // 绳索悬空力计算
    double angle_rad_4 = atan((d1 + d2 + d3 - Yd)/LB);
    G = LB * 1.2;
    double F5_1 = G / sin(angle_rad_4);
    double Ff2 = (l4 - LB) * 0.12 * g * u2;
    double F5_2 = Ff2 / cos(angle_rad_4);
    double F5 = F5_1 + F5_2;

    // 合力转矩
    double angle_rad_5 = atan(d3 / l3);
    double F2 = F5 * cos(angle_rad_5) / cos(angle_rad_3);
    TG = F2 * R + Tf + Tc;

    // 百分比表示
    TG_1 = TG / 1.27 / 20 * 100;

    // 输出调试信息
    qDebug() << "TG =" << TG;
    qDebug() << "F2 =" << F2;
    qDebug() << "F5 =" << F5;
    qDebug() << "TG_1 =" << TG_1 << "%";
}

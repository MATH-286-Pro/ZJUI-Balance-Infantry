#ifndef _ZJUI_BALANCE_
#define _ZJUI_BALANCE_

// 底盘参数 ZJUI
#define THIGH_LEN 0.15f            // 上腿
#define CALF_LEN 0.27f             // 下腿
#define JOINT_DISTANCE 0.15f       // 关节间距
#define WHEEL_RADIUS 0.0622f       // 轮子半径
#define LIMIT_LINK_RAD 0.15149458  // 初始限位角度,见ParamAssemble  //未修改
#define WHEEL_DISTANCE 0.424f      // 轮子间距


// 腿结构体
typedef struct
{
    // joint
    float phi1_w, phi4_w, phi2_w, phi5_w; // phi2_w used for calc real wheel speed
    float T_back, T_front;
    // link angle, phi1-ph5, phi5 is pod angle
    float phi1, phi2, phi3, phi4, phi5;
    // wheel
    float w_ecd;      // 轮电机角速度    (电机坐标)
    float wheel_dist; // 轮子的水平位移  (世界坐标)
    float wheel_w;    // 轮子的角速度    (世界坐标)
    float body_v;     // 髋关节水平速度  (世界坐标)
    float T_wheel;
    // pod
    float theta, theta_w; // 杆和垂直方向的夹角,为控制状态之一
    float leg_len, legd;
    float height, height_v;
    float F_leg, T_hip;
    float target_len;

    float coord[6]; // xb yb xc yc xd yd

    float wheel_out[7];
    float hip_out[7];
} LinkNPodParam;

// 底盘结构体
typedef struct
{
    float vel, target_v;        // 底盘速度
    float vel_m;                // 底盘速度测量值
    float vel_predict;          // 底盘速度预测值
    float vel_cov;              // 速度方差
    float acc, acc_m, acc_last; // 水平方向加速度,用于计算速度预测值

    float dist, target_dist;   // 底盘位移距离
    float yaw, wz, target_yaw; // yaw角度和底盘角速度
    float pitch, pitch_w;      // 底盘俯仰角度和角速度
    float roll, roll_w;        // 底盘横滚角度和角速度
} ChassisParam;

#endif

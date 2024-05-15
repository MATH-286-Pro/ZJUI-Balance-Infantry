#ifndef _BALANCE_
#define _BALANCE_

#include "ins_task.h"

// 底盘参数 ZJUI
#define THIGH_LEN 0.15f            // 上腿
#define CALF_LEN 0.27f             // 下腿
#define JOINT_DISTANCE 0.15f       // 关节间距
#define WHEEL_RADIUS 0.0622f       // 轮子半径
#define LIMIT_LINK_RAD 0.15149458  // 初始限位角度,见ParamAssemble  //未修改
#define WHEEL_DISTANCE 0.424f      // 轮子间距

// C板距离中心距离，因为要计算机体中心的加速度，如果C板不在中心会产生额外加速度
// IMU距离中心的距离
#define CENTER_IMU_W 0.068f
#define CENTER_IMU_L 0.070f
#define CENTER_IMU_H 0.039f

// 滤波参数
#define VEL_PROCESS_NOISE 25  // 速度过程噪声
#define VEL_MEASURE_NOISE 800 // 速度测量噪声

// 添加板凳模型函数
void stand_task_init();
void stand_task_start(INS_t *INS, float Vel_Forward, float Vel_Turn);

#endif

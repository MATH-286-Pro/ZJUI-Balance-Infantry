#ifndef JOINT_H
#define JOINT_H

#include "main.h"
#include "unitreeA1_cmd.h"

// 宏定义
#define PI 3.1415926535f
#define DGR2RAD PI/180
#define RAD2DGR 180/PI
#define False 0
#define True 1

// 设定关节电机零点初始位置
extern float zero_left_ID0;
extern float zero_left_ID1;
extern float zero_right_ID0;
extern float zero_right_ID1;

// 电机数据发送结构体
extern motor_send_t cmd_left;         // 左腿一号电机数据体
extern motor_send_t cmd_right;        // 右腿一号电机数据体

extern motor_recv_t Date_left;        // 左腿电机接收数据体
extern motor_recv_t id00_left_date;   // 左腿00号电机接收数据体
extern motor_recv_t id01_left_date;   // 左腿01号电机接收数据体
extern motor_recv_t id02_left_date;   // 左腿02号电机接收数据体


// 电机零点自检
int Joint_Zero_OK(void);

// 电机零点获取 (零点位置 = 上电位置)
void Joint_Zero_init_Type1(void);

// void Joint_Zero_init_Type2(void);

#endif // !JOINT_H

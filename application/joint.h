#ifndef JOINT_H
#define JOINT_H

#include "main.h"
#include "A1_motor_drive.h"

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
extern motor_send_t MotorA1_send_left;         // 左腿一号电机数据体
extern motor_send_t MotorA1_send_right;        // 右腿一号电机数据体

extern motor_recv_t Date_left;                // 左腿电机接收数据体
extern motor_recv_t MotorA1_recv_left_id00;   // 左腿00号电机接收数据体
extern motor_recv_t MotorA1_recv_left_id01;   // 左腿01号电机接收数据体

extern motor_recv_t Date_right;                // 右腿电机接收数据体
extern motor_recv_t MotorA1_recv_right_id00;   // 右腿00号电机接收数据体
extern motor_recv_t MotorA1_recv_right_id01;   // 右腿01号电机接收数据体

extern uint8_t STOP; // 急停状态

// 定义底盘参数
// 记录电机零位
typedef struct {
    float zero_l_ID0; // 减速后的角度制 零点
    float zero_l_ID1;
    float zero_r_ID0;
    float zero_r_ID1;

} Chassis_ME_t;

// 底盘初始化
Chassis_ME_t *Chassis_Init();

// 电机零点自检
int Joint_Zero_OK(void);

// 电机零点获取 (零点位置 = 上电位置)
void Joint_Zero_init_Type1(void);

// 电机零点获取 (零点位置 = 限位位置)
void Joint_Zero_init_Type2(void);

// 存在问题
void Joint_GOTO_zero(void);

// 监控电机位置与力矩状态
void Joint_Monitor(void);

// 底盘位置控制
void Joint_Position_Control(float Pos_Front, float Pos_Back);

// 底盘速度控制
void Joint_Speed_Control(float Speed_Front, float Speed_Back);

void Joint_Full_Position_Control(float Pos_Front_Left, float Pos_Front_Right, float Pos_Back_Left, float Pos_Back_Right);

// 离地检测 (测试)
// uint8_t Joint_IsOn_Ground();


#endif // !JOINT_H

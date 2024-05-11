#ifndef __WHEEL_H_
#define __WHEEL_H_

#include "MI_motor_drive.h"
#include "can.h"
#include "can_test.h"
#include "cmsis_os.h"

// 宏定义
#define NOT 0

// 定义变量
extern MI_Motor_s MI_Motor_ID1;              // 定义小米电机结构体1
extern MI_Motor_s MI_Motor_ID2;              // 定义小米电机结构体2


// 定义函数
uint8_t Wheel_Init_OK();
void Wheel_Init();
void Wheel_Torque_Control(float Torque_L, float Torque_R);
void Wheel_Speed_Control(float Speed_L, float Speed_R);


#endif

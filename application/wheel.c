#include "wheel.h"

uint8_t Wheel_Init_OK()
{
    if (MI_Motor_ID1.RxCAN_info.motor_id != 0 && MI_Motor_ID2.RxCAN_info.motor_id != 0)
    {return 1;}
    else {return 0;}
}

void Wheel_Init()
{   MI_motor_Init(&MI_Motor_ID1,&MI_CAN_1,1); // 将MI_CAN_1，ID=1传入小米结构体 
    MI_motor_Init(&MI_Motor_ID2,&MI_CAN_1,2); // 将MI_CAN_1，ID=2传入小米结构体 
    MI_motor_Enable(&MI_Motor_ID1);           // 通过发送小米结构体 data=00000000 电机使能
    MI_motor_Enable(&MI_Motor_ID2);           // 通过发送小米结构体 data=00000000 电机使能

    while(Wheel_Init_OK() == NOT)
    {  
        MI_motor_Enable(&MI_Motor_ID1);           // 通过发送小米结构体 data=00000000 电机使能
        MI_motor_Enable(&MI_Motor_ID2);           // 通过发送小米结构体 data=00000000 电机使能
        osDelay(10);
    }
}

/**
  * @brief          轮电机力矩控制 正值为前进，负值为后退
  * @param[in]      Torque_L: 左轮力矩
  * @param[in]      Torque_R: 右轮力矩
  */
void Wheel_Torque_Control(float Torque_L, float Torque_R)
{
    MI_motor_TorqueControl(&MI_Motor_ID2, -Torque_L); // 左轮
    MI_motor_TorqueControl(&MI_Motor_ID1, +Torque_R); // 右轮
}

/**
  * @brief          轮电机速度控制 正值为前进，负值为后退
  * @param[in]      Speed_L: 左轮速度
  * @param[in]      Speed_R: 右轮速度
  */void Wheel_Speed_Control(float Speed_L, float Speed_R)
{
    MI_motor_SpeedControl(&MI_Motor_ID2, -Speed_L,1); // 左轮
    MI_motor_SpeedControl(&MI_Motor_ID1, +Speed_R,1); // 右轮
}


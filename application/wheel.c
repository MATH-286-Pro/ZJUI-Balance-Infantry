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


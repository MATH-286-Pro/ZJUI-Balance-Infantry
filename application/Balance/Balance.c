// User includes
#include "Balance.h"

#include "bsp_dwt.h"
#include "ins_task.h"
#include "general_def.h" // 通用参数，比如pi
#include "Debug_Tool.h"
#include "rc.h"
#include "A1_motor_drive.h"
#include "MI_motor_drive.h"
#include "wheel.h"
#include "joint.h"


// User define variables
// 关节电机变量
extern motor_send_t MotorA1_send_left;        // 左腿一号电机数据体
extern motor_send_t MotorA1_send_right;       // 右腿一号电机数据体
extern motor_recv_t MotorA1_recv_left_id00;   // 左腿00号电机接收数据体
extern motor_recv_t MotorA1_recv_left_id01;   // 左腿01号电机接收数据体
extern motor_recv_t MotorA1_recv_right_id00;  // 右腿00号电机接收数据体
extern motor_recv_t MotorA1_recv_right_id01;  // 右腿01号电机接收数据体
// 默认电机零位
extern float zero_left_ID0;
extern float zero_left_ID1;
extern float zero_right_ID0;
extern float zero_right_ID1;
// 轮电机变量
extern MI_Motor_s MI_Motor_ID1;              // 定义小米电机结构体1
extern MI_Motor_s MI_Motor_ID2;              // 定义小米电机结构体2
// IMU变量
extern INS_t INS;         

extern RC_Type rc;        





// 板凳模型算法
#include "pid.h"

#define mm 0.001f
#define R_Wheel  124*mm  // 轮子半径

#define IsInDeadZone(measure, target, DeadZone) (((measure - target)> -DeadZone && (measure - target) < DeadZone) ? target : measure)
#define IsInDeadZoneSign(measure, target, DeadZone) (((measure - target)> -DeadZone && (measure - target) < DeadZone) ? 0 : 1)

float target_pitch = 0.0f; // 测试实际数值角度
float Vel_L = 0.0f;
float Vel_R = 0.0f;
float Vel_Diff = 0.0f;
float Vel_measure;              // 平均速度 = (左 + 右) / 2
float Vel_measure_last;         // 上一次速度
float Vel_measure_mod;          // 速度修正
float Vel_print = 0.0f;         // 用于打印
uint8_t DZ_SIGN_VEL = 1;        // 速度环死区标志
float DeadZone_Vel  = 0.16f;    // 速度环死区
float DeadZone_TURN = 0.5f;     // 转向环死区

pid_type_def PID_Balance_only;  // 直立环 PID 结构体
pid_type_def PID_Balance;       // 直立环 PID 结构体
pid_type_def PID_VEL_UP;        // 速度环 PID 结构体 (±10°都非常稳定)
pid_type_def PID_TURN;          // 转向环 PID 结构体

void stand_task_init()
{   
    // 直立环参数
    static const float PID_ARG[3] = {0.8f, 0.0f, 17.0f};     
    static const float PID_MAX_OUT  = 3.0f; // 小米电机输峰值扭矩为 12Nm
    PID_init(&PID_Balance, PID_POSITION, PID_ARG, PID_MAX_OUT, 4.0f);
    PID_init(&PID_Balance_only, PID_POSITION, PID_ARG, PID_MAX_OUT, 4.0f);

    // 速度环参数
    static const float PID_VEL_UP_ARG[3] = {1.8f, 0.015f, 0.0f};    
    PID_init(&PID_VEL_UP, PID_POSITION, PID_VEL_UP_ARG, 8.0f, 0.3f);    // MAX_IOUT = 0.3f    

    // 转向环
    static const float PID_TURN_ARG[3] = {5.0f, 0.0f, 1.0f};      
    PID_init(&PID_TURN, PID_POSITION, PID_TURN_ARG, 1.0f, 0.0f);  

    Vel_measure_last = 0.0;
    Vel_measure = 0.0;
}

void stand_task_start(INS_t *INS, float RC_Forward, float RC_Turn)
{   
    // 速度环计算
    Vel_measure_last = Vel_measure; // 用于滤波
    Wheel_Speed_Read(&Vel_L, &Vel_R);                                       // 读取轮速
    Vel_measure     = 0.5*(Vel_L * R_Wheel + Vel_R * R_Wheel);              // 车速 = (左轮速 + 右轮速) / 2
    Vel_measure     = Vel_measure - INS->Gyro[X] * R_Wheel;                 // 轮速度修正

    Vel_measure     = Vel_measure * 0.3f + Vel_measure_last * 0.7f;                  // 速度滤波
    Vel_print       = Vel_measure;                                                  // 用于打印
    DZ_SIGN_VEL = IsInDeadZoneSign(Vel_measure,RC_Forward*4.0f, DeadZone_Vel);       // 速度环死区标志
    PID_calc(&PID_VEL_UP,   Vel_measure, RC_Forward*4.0f);                       // 计算 平衡 速度环 输出

    // 直立环计算
    // PID_calc(&PID_Balance, INS->Roll, 0.0f);                       // 计算 PID 输出
    PID_calc(&PID_Balance, INS->Roll, -DZ_SIGN_VEL*PID_VEL_UP.out);   // 计算 PID 输出
    PID_calc(&PID_Balance_only, INS->Roll, 0.0f);                     // 计算 PID 输出

    // 转向环计算
    Vel_Diff = Vel_L - Vel_R;                                // 左右轮速差
    Vel_Diff = IsInDeadZone(Vel_Diff, 0.0f, DeadZone_TURN);  // 转向环死区
    PID_calc(&PID_TURN, Vel_Diff, RC_Turn*20.0f);            // 计算 转向环 输出



    // 力矩输出
    static float Critical_Angle = 8.0f; // 临界角度
    if (INS->Roll > -Critical_Angle && INS->Roll < Critical_Angle)
    {Wheel_Torque_Control(PID_Balance.out + PID_TURN.out,   // 左轮
                          PID_Balance.out - PID_TURN.out);} // 右轮
    else
    {Wheel_Torque_Control(PID_Balance_only.out + PID_TURN.out,   // 左轮
                          PID_Balance_only.out - PID_TURN.out);}

}

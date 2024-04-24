// User includes
#include "ZJUI_balance.h"
#include "ZJUI_linkNleg.h" //ZJUI_linkNleg中包含 arm_math.h 后出现 __SMMLA 重定义问题
// #include "ZJUI_LQR_calc.h"
// #include "ZJUI_speed_estimation.h"

#include "unitreeA1_cmd.h"
#include "MI_motor_drive.h"
#include "INS_task.h"
#include "general_def.h" // 通用参数，比如pi

// User define variables
// 关节电机变量
extern motor_recv_t MotorA1_recv_left_id00;   // 左腿00号电机接收数据体
extern motor_recv_t MotorA1_recv_left_id01;   // 左腿01号电机接收数据体
extern motor_recv_t MotorA1_recv_right_id00;  // 右腿00号电机接收数据体
extern motor_recv_t MotorA1_recv_right_id01;  // 右腿01号电机接收数据体
// 轮电机变量
extern MI_Motor_s MI_Motor_ID1;              // 定义小米电机结构体1
extern MI_Motor_s MI_Motor_ID2;              // 定义小米电机结构体2
// IMU变量
extern INS_t INS;         
// 两个腿的参数,0为左腿,1为右腿
static LinkNPodParam l_side, r_side;    
static ChassisParam chassis;



// 任务
// 查看Link2Leg() 解算是否正确
void BalanceTask()
{   
    // 参数组装
    ParamAssemble();

    // 五连杆换算
    Link2Leg(&l_side, &chassis);
    Link2Leg(&r_side, &chassis);

}


// 参数组装
void ParamAssemble()
{
    // 传入电机参数
    l_side.phi1  = MotorA1_recv_left_id00.Pos;     l_side.phi1_w = MotorA1_recv_left_id00.W;
    l_side.phi4  = MotorA1_recv_left_id01.Pos;     l_side.phi4_w = MotorA1_recv_left_id01.W;
    l_side.w_ecd = MI_Motor_ID2.RxCAN_info.speed;

    r_side.phi1  = MotorA1_recv_right_id00.Pos;    r_side.phi1_w = MotorA1_recv_right_id00.W;
    r_side.phi4  = MotorA1_recv_right_id01.Pos;    r_side.phi4_w = MotorA1_recv_right_id01.W; 
    r_side.w_ecd = MI_Motor_ID1.RxCAN_info.speed;

    // 传入IMU参数
    chassis.yaw   = INS.Yaw * DEGREE_2_RAD;    chassis.wz      = INS.Gyro[2];  
    chassis.pitch = INS.Pitch * DEGREE_2_RAD;  chassis.pitch_w = INS.Gyro[0];  
    chassis.roll  = INS.Roll * DEGREE_2_RAD;   chassis.roll_w  = INS.Gyro[1];  

}
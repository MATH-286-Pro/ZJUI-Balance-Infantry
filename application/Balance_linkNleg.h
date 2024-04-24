#include "Balance.h"
#include "math.h"
#include "user_lib.h"
#include "arm_math.h"


/* 计算的T_hip和F_Leg映射为关节电机输出 */
void VMCProject(LinkNPodParam *p)
{
    float s23 = msin(p->phi2 - p->phi3);
    float phi25 = p->phi2 - p->phi5;
    float phi35 = p->phi3 - p->phi5;
    float F_m_L = p->F_leg * p->leg_len;
    p->T_back = (THIGH_LEN * msin(p->phi1 - p->phi2) * (p->T_hip * mcos(phi35) - F_m_L * msin(phi35))) / (p->leg_len * s23);
    p->T_front = (THIGH_LEN * msin(p->phi3 - p->phi4) * (p->T_hip * mcos(phi25) - F_m_L * msin(phi25))) / (p->leg_len * s23);
}

/**
 * @brief 根据关节角度和角速度,计算单杆长度和角度以及变化率
 *
 * @note 右侧视图
 *  ___x
 * |    1  _____  4
 * |y     /     \
 *      2 \     / 3
 *         \   /
 *          \./
 *           5
 * @param p 5连杆和腿的参数
 */
void Link2Leg(LinkNPodParam *p, ChassisParam *chassis)
{
    float xD, yD, xB, yB, BD, A0, B0, xC, yC;
    p->coord[4] = xD = JOINT_DISTANCE + THIGH_LEN * mcos(p->phi4);
    p->coord[5] = yD = THIGH_LEN * msin(p->phi4);
    p->coord[0] = xB = THIGH_LEN * mcos(p->phi1);
    p->coord[1] = yB = THIGH_LEN * msin(p->phi1);
    BD = powf(xD - xB, 2) + powf(yD - yB, 2);
    A0 = 2 * CALF_LEN * (xD - xB);
    B0 = 2 * CALF_LEN * (yD - yB);
    p->phi2 = 2 * atan2f(B0 + Sqrt(powf(A0, 2) + powf(B0, 2) - powf(BD, 2)), A0 + BD);
    p->coord[2] = xC = xB + CALF_LEN * mcos(p->phi2);
    p->coord[3] = yC = yB + CALF_LEN * msin(p->phi2);

    p->phi5 = atan2f(yC, xC - JOINT_DISTANCE / 2);
    p->leg_len = Sqrt(powf(xC - JOINT_DISTANCE / 2, 2) + powf(yC, 2));
    p->phi3 = atan2f(yC - yD, xC - xD);             // 稍后用于计算VMC
    p->theta = p->phi5 - 0.5 * PI - chassis->pitch; // 确定方向
    p->height = p->leg_len * mcos(p->theta);

    static float predict_dt = 0.0001f;
    float phi1_pred = p->phi1 + p->phi1_w * predict_dt; // 预测下一时刻的关节角度(利用关节角速度)
    float phi4_pred = p->phi4 + p->phi4_w * predict_dt;
    // 重新计算腿长和腿角度
    xD = JOINT_DISTANCE + THIGH_LEN * mcos(phi4_pred);
    yD = THIGH_LEN * msin(phi4_pred);
    xB = 0 + THIGH_LEN * mcos(phi1_pred);
    yB = THIGH_LEN * msin(phi1_pred);
    BD = powf(xD - xB, 2) + powf(yD - yB, 2);
    A0 = 2 * CALF_LEN * (xD - xB);
    B0 = 2 * CALF_LEN * (yD - yB);
    float phi2_pred = 2 * atan2f(B0 + Sqrt(powf(A0, 2) + powf(B0, 2) - powf(BD, 2)), A0 + BD);
    xC = xB + CALF_LEN * mcos(phi2_pred);
    yC = yB + CALF_LEN * msin(phi2_pred);
    float phi5_pred = atan2f(yC, xC - JOINT_DISTANCE / 2);
    // 差分计算腿长变化率和腿角速度
    p->phi2_w = (phi2_pred - p->phi2) / predict_dt; // 稍后用于修正轮速
    p->phi5_w = (phi5_pred - p->phi5) / predict_dt;
    p->legd = (Sqrt(powf(xC - JOINT_DISTANCE / 2, 2) + powf(yC, 2)) - p->leg_len) / predict_dt;
    p->theta_w = ((phi5_pred - 0.5 * PI - chassis->pitch - p->theta) / predict_dt); // 可以不考虑机体? -predict_dt*chassis.pitch_w
    p->height_v = p->legd * mcos(p->theta) - p->leg_len * msin(p->theta) * p->theta_w;
}

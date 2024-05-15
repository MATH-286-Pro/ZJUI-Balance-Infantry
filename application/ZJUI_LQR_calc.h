#include "ZJUI_balance.h"
#include "ZJUI_linkNleg.h"


static ChassisParam chassis;

/**
 * @brief 根据状态反馈计算当前腿长,查表获得LQR的反馈增益,并列式计算LQR的输出
 * @note 得到的腿部力矩输出还要经过综合运动控制系统补偿后映射为两个关节电机输出
 *
 */
static void CalcLQR(LinkNPodParam *p)
{
    static float k[12][3] = {60.071162, -57.303242, -8.802552, -0.219882, -1.390464, -0.951558, 32.409644, -23.635877, -9.253521, 12.436266, -10.318639, -7.529540, 135.956804, -124.044032, 36.093582, 13.977819, -12.325081, 3.766791, 32.632159, -39.012888, 14.483707, 7.204778, -5.973109, 1.551763, 78.723013, -73.096567, 21.701734, 63.798027, -55.564993, 15.318437, -195.813486, 140.134182, 60.699132, -18.357761, 13.165559, 2.581731};
    float T[2] = {0}; // 0 T_wheel  1 T_hip
    float l = p->leg_len;
    float lsqr = l * l;
    // float dist_limit = abs(chassis.target_dist - chassis.dist) > MAX_DIST_TRACK ? sign(chassis.target_dist - chassis.dist) * MAX_DIST_TRACK : (chassis.target_dist - chassis.dist); // todo设置值
    // float vel_limit = abs(chassis.target_v - chassis.vel) > MAX_VEL_TRACK ? sign(chassis.target_v - chassis.vel) * MAX_VEL_TRACK : (chassis.target_v - chassis.vel);
    for (uint8_t i = 0; i < 2; ++i)
    {
        uint8_t j = i * 6;
        T[i] = (k[j + 0][0] * lsqr + k[j + 0][1] * l + k[j + 0][2]) * -p->theta +
               (k[j + 1][0] * lsqr + k[j + 1][1] * l + k[j + 1][2]) * -p->theta_w +
               (k[j + 2][0] * lsqr + k[j + 2][1] * l + k[j + 2][2]) * (chassis.target_dist - chassis.dist) +
               (k[j + 3][0] * lsqr + k[j + 3][1] * l + k[j + 3][2]) * (chassis.target_v - chassis.vel) +
               (k[j + 4][0] * lsqr + k[j + 4][1] * l + k[j + 4][2]) * -chassis.pitch +
               (k[j + 5][0] * lsqr + k[j + 5][1] * l + k[j + 5][2]) * -chassis.pitch_w;
    }
    p->T_wheel = T[0];
    p->T_hip = T[1];
}

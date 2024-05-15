#ifndef __ZJUI_SPEED_ESTIMATION_H
#define __ZJUI_SPEED_ESTIMATION_H

#include "ZJUI_balance.h"
#include "user_lib.h"
#include "general_def.h"
#include "ins_task.h" 

#define EST_FINAL_LPF 0.005f // 最终速度的低通滤波系数
static KalmanFilter_t kf;

/**
 * @brief 底盘为右手系
 *
 *    ^ y            左轮     右轮
 *    |               |      |  前
 *    |_____ >  x     |------|
 *    z 轴从屏幕向外   |      |  后
 */

void SpeedEstInit()
{   
    // 使用kf同时估计速度和加速度
    // Kalman_Filter_Init(&kf, 2, 0, 2);
    // float F[4] = {1, 0.001, 0, 1};
    // float Q[4] = {VEL_PROCESS_NOISE, 0, 0, ACC_PROCESS_NOISE};
    // float R[4] = {VEL_MEASURE_NOISE, 0, 0, ACC_MEASURE_NOISE};
    // float P[4] = {100000, 0, 0, 100000};
    // float H[4] = {1, 0, 0, 1};
    // memcpy(kf.F_data, F, sizeof(F));
    // memcpy(kf.Q_data, Q, sizeof(Q));
    // memcpy(kf.R_data, R, sizeof(R));
    // memcpy(kf.P_data, P, sizeof(P));
    // memcpy(kf.H_data, H, sizeof(H));
}

/**
 * @brief 使用卡尔曼滤波估计底盘速度
 * @todo 增加w和dw的滤波,当w和dw均小于一定值时,不考虑dw导致的角加速度
 *
 * @param lp 左侧腿
 * @param rp 右侧腿
 * @param cp 底盘
 * @param imu imu数据
 * @param delta_t 更新间隔
 */
void SpeedEstimation(LinkNPodParam *lp, LinkNPodParam *rp, ChassisParam *cp, INS_t *imu, float delta_t)
{
    // 修正轮速和距离
    lp->wheel_w = lp->w_ecd + lp->phi2_w - cp->pitch_w; // 减去和定子固连的phi2_w
    rp->wheel_w = rp->w_ecd + rp->phi2_w - cp->pitch_w;

    // 直接使用轮速反馈,不进行速度融合
    // cp->vel = (lp->wheel_w + rp->wheel_w) * WHEEL_RADIUS / 2;
    // cp->dist = cp->dist + cp->vel * delta_t;

    // 以轮子为基点,计算机体两侧髋关节处的速度
    // 速度计算 = 轮ω·r + 腿ω·l + 腿长变化率·sin(θ)
    lp->body_v = lp->wheel_w * WHEEL_RADIUS + lp->leg_len * lp->theta_w + lp->legd * msin(lp->theta);
    rp->body_v = rp->wheel_w * WHEEL_RADIUS + rp->leg_len * rp->theta_w + rp->legd * msin(rp->theta);
    cp->vel_m = (lp->body_v + rp->body_v) / 2; // 机体速度(平动)为两侧速度的平均值

    // 扣除旋转导致的向心加速度和角加速度*R
    float *gyro = imu->Gyro, *dgyro = imu->dgyro;
    static float yaw_ddwrNwwr, yaw_p_ddwrNwwr, pitch_ddwrNwwr;
    static float macc_y, macc_z;                                // 补偿后的实际平动加速度,机体系前进方向和竖直方向
    yaw_ddwrNwwr   = powf(gyro[Z], 2) * CENTER_IMU_W - dgyro[Z] * CENTER_IMU_L; // yaw旋转导致motion_acc[1]的额外加速度(机体前后方向)
    yaw_p_ddwrNwwr = powf(gyro[X], 2) * CENTER_IMU_L + dgyro[X] * CENTER_IMU_H; // pitch旋转导致motion_acc[1]的额外加速度(机体前后方向)
    pitch_ddwrNwwr = powf(gyro[X], 2) * CENTER_IMU_H + dgyro[X] * CENTER_IMU_L; // pitch旋转导致motion_acc[2]的额外加速度(机体竖直方向)

    macc_y = -imu->MotionAccel_b[Y] - yaw_ddwrNwwr - yaw_p_ddwrNwwr; // Y加速度修正
    macc_z = +imu->MotionAccel_b[Z] - pitch_ddwrNwwr;                // Z加速度修正


    // 三角函数计算水平加速度
    float pitch = imu->Pitch * DEGREE_2_RAD;
    cp->acc_last = cp->acc_m; 
    cp->acc_m = macc_y * mcos(pitch) + macc_z * msin(pitch); // 绝对系下的平动加速度,即机体系下的加速度投影到绝对系

    // for debug 对比修正前后的加速度
    static float ry, rz, rawaa; 
    ry = -imu->MotionAccel_b[Y];
    rz = +imu->MotionAccel_b[Z];
    rawaa = ry * mcos(pitch) + rz * msin(pitch);

    // 使用kf同时估计加速度和速度,滤波更新
    // kf.MeasuredVector[0] = cp->vel_m;
    // kf.MeasuredVector[1] = cp->acc_m;
    // kf.F_data[1] = delta_t; // 更新F矩阵
    // Kalman_Filter_Update(&kf);
    // cp->vel = kf.xhat_data[0];
    // cp->acc = kf.xhat_data[1];

    // 融合加速度计的数据和机体速度
    static float f, k, prior, measure, cov;
    f = (cp->acc_m + cp->acc_last) / 2;    // 速度梯形积分
    prior = cp->vel + f * delta_t; // x' = Fx,先验
    cp->vel_predict = prior;
    measure = cp->vel_m;
    cov = cp->vel_cov + VEL_PROCESS_NOISE * delta_t; // P' = P + Q ,先验协方差
    cp->vel_cov = cov;
    k = cov / (cov + VEL_MEASURE_NOISE);     // K = P'/(P'+R),卡尔曼增益

    // 速度计算
    cp->vel = prior + k * (measure - prior); // x^ = x'+K(z-x'),后验估计
    cp->vel_cov *= (1 - k);                  // P^ = (1-K)P',后验协方差
    VAL_LIMIT(cp->vel_cov, 0.01, 100);       // 协方差限幅
    cp->dist = cp->dist + cp->vel * delta_t;  //--------------------------------里程计计算位移-----------------------------------------
}

#endif // __ZJUI_SPEED_ESTIMATION_H
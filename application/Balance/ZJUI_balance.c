#include "ZJUI_balance.h"
#include "ZJUI_linkNleg.h" //ZJUI_linkNleg中包含 arm_math.h 后出现 __SMMLA 重定义问题
// #include "ZJUI_LQR_calc.h"
// #include "ZJUI_speed_estimation.h"



// 两个腿的参数,0为左腿,1为右腿
static LinkNPodParam l_side, r_side; // syh  phi5
static ChassisParam chassis;


#include "main.h"
#include "buzzer.h"
#include "tim.h"

void Buzzer_beep(void) {
    HAL_TIM_Base_Start(&htim4);               // 开启TIM4
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);  // 开启TIM4 PWM 蜂鸣器
    __HAL_TIM_PRESCALER(&htim4, 8);           // 设置TIM4 预分频 调整音色
    HAL_Delay(100);                           // 延时0.1s
    HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);   // 关闭TIM4 PWM
}

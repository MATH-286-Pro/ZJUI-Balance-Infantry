#include "joint.h"

// 默认电机初始零点
float zero_left_ID0 = 1000.0f;
float zero_left_ID1 = 1000.0f;
float zero_right_ID0 = 1000.0f;
float zero_right_ID1 = 1000.0f;


// 电机零点自检
int Joint_Zero_OK() {
    // 检查所有零位是否都在(-180, 180)范围内
    // 一般来说 left_ID0 零位不可能等于 right_ID0 零位
    if ((zero_left_ID0 > -180 && zero_left_ID0 < 180) &&
        (zero_right_ID0 > -180 && zero_right_ID0 < 180) &&
        (zero_left_ID1 > -180 && zero_left_ID1 < 180) &&
        (zero_right_ID1 > -180 && zero_right_ID1 < 180) &&
        (zero_left_ID0 != zero_right_ID0) &&
        (zero_left_ID1 != zero_right_ID1)) {
        return 1;  // 如果所有零位都在范围内，则返回 true
    }
    return 0;  // 否则返回 false
}

// 电机零点获取 (初始位置 = 上电位置)
void Joint_Zero_init_Type1()
{
  // 电机零位 默认为1000，为了循环判断所以这么写 
  // 电机零位 定义在最上面
  // 使用while循环确保0位正确

  // 自检不通过 红灯亮起
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_SET); //

  while (Joint_Zero_OK() == False) {

      modfiy_torque_cmd(&cmd_left, 0, 0);    modfiy_torque_cmd(&cmd_right, 0, 0);
      unitreeA1_rxtx(&huart1);               unitreeA1_rxtx(&huart6);
      zero_left_ID0  = (float) id00_left_date.Pos * RAD2DGR / 9.1f;
      zero_right_ID0 = (float) id00_right_date.Pos * RAD2DGR / 9.1f;
      osDelay(2);

      modfiy_torque_cmd(&cmd_left, 1, 0);    modfiy_torque_cmd(&cmd_right, 1, 0);
      unitreeA1_rxtx(&huart1);               unitreeA1_rxtx(&huart6);
      zero_left_ID1  = (float) id01_left_date.Pos * RAD2DGR / 9.1f;
      zero_right_ID1 = (float) id01_right_date.Pos * RAD2DGR / 9.1f;
      osDelay(2);}

  // 自检成功 红灯熄灭
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET); //

}

// // 电机零位获取 (初始位置 = 上限位位置)
// void Jpint_Zero_Init_Type2()
// {

// }

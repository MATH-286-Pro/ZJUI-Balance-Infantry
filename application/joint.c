#include <stdlib.h>
#include <stdio.h>
#include "joint.h"
#include "cmsis_os.h"


// 默认电机初始零点
float zero_left_ID0  = 0.0f;
float zero_left_ID1  = 0.0f;
float zero_right_ID0 = 0.0f;
float zero_right_ID1 = 0.0f;

uint8_t STOP = False;

static float home_speed  = 1.1f;  // 减速后角速度 rad/s
static float home_torque = 2.7f;  // 减速后力矩 Nm
 float UP_LIMIT    = 20.0f; // 减速后角度 °
 float DOWN_LIMIT  = 80.0f; // 减速后角度 °
 float TOLERANCE   = -1.0f;  // 容差 °

// 电机零点自检
int Joint_Zero_OK() {
    // 检查所有零位是否都在(-180, 180)范围内
    // 一般来说 left_ID0 零位不可能等于 right_ID0 零位
    if ((zero_left_ID0 > -180 && zero_left_ID0 < 180) && zero_left_ID0 != 0 &&
        (zero_right_ID0 > -180 && zero_right_ID0 < 180) && zero_right_ID0 != 0 &&
        (zero_left_ID1 > -180 && zero_left_ID1 < 180) && zero_left_ID1 != 0 &&
        (zero_right_ID1 > -180 && zero_right_ID1 < 180) && zero_left_ID0 != 0 &&
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

      modfiy_torque_cmd(&MotorA1_send_left, 0, 0);    modfiy_torque_cmd(&MotorA1_send_right, 0, 0);
      unitreeA1_rxtx(&huart1);               unitreeA1_rxtx(&huart6);
      zero_left_ID0  = (float) MotorA1_recv_left_id00.Pos ;
      zero_right_ID0 = (float) MotorA1_recv_right_id00.Pos ;
      osDelay(2);

      modfiy_torque_cmd(&MotorA1_send_left, 1, 0);    modfiy_torque_cmd(&MotorA1_send_right, 1, 0);
      unitreeA1_rxtx(&huart1);               unitreeA1_rxtx(&huart6);
      zero_left_ID1  = (float) MotorA1_recv_left_id01.Pos ;
      zero_right_ID1 = (float) MotorA1_recv_right_id01.Pos ;
      osDelay(2);
      
      osDelay(20);
      }

  // 自检成功 红灯熄灭
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET); //

}

// 电机零位获取 (初始位置 = 限位位置)
void Joint_Zero_init_Type2()
{
// 自检不通过 红灯亮起
HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_SET); //

while (Joint_Zero_OK() == False) {

    modfiy_speed_cmd(&MotorA1_send_left,  0, -home_speed);    
    modfiy_speed_cmd(&MotorA1_send_right, 0, +home_speed);
    unitreeA1_rxtx(&huart1);                           unitreeA1_rxtx(&huart6);
    if ((MotorA1_recv_left_id00.T)  <= -home_torque) {zero_left_ID0  = (float) MotorA1_recv_left_id00.Pos + UP_LIMIT;} // zero_left_ID0 是减速后的角度 (不是弧度)
    if ((MotorA1_recv_right_id00.T) >= +home_torque) {zero_right_ID0 = (float) MotorA1_recv_right_id00.Pos - UP_LIMIT;}
    osDelay(1);

    modfiy_speed_cmd(&MotorA1_send_left,  1, +home_speed);    
    modfiy_speed_cmd(&MotorA1_send_right, 1, -home_speed);
    unitreeA1_rxtx(&huart1);                           unitreeA1_rxtx(&huart6);
    if ((MotorA1_recv_left_id01.T)  >= +home_torque) {zero_left_ID1  = (float) MotorA1_recv_left_id01.Pos - UP_LIMIT;}
    if ((MotorA1_recv_right_id01.T) <= -home_torque) {zero_right_ID1 = (float) MotorA1_recv_right_id01.Pos + UP_LIMIT;}
    osDelay(1);
    }
HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET); //

}


// 检测是否超过上限位 // right 转换器有问题
void Joint_Monitor()
{   
    if (((MotorA1_recv_left_id00.Pos  - zero_left_ID0)  <= -(UP_LIMIT+TOLERANCE) || (MotorA1_recv_left_id00.Pos - zero_left_ID0) >= +(DOWN_LIMIT+TOLERANCE)) && zero_left_ID0 != 0)
        {STOP = True;}
    if (((MotorA1_recv_left_id01.Pos  - zero_left_ID1)  >= +(UP_LIMIT+TOLERANCE) || (MotorA1_recv_left_id01.Pos - zero_left_ID1) <= -(DOWN_LIMIT+TOLERANCE)) && zero_left_ID1 != 0)
        {STOP = True;}
    if (((MotorA1_recv_right_id00.Pos - zero_right_ID0) >= +(UP_LIMIT+TOLERANCE) || (MotorA1_recv_right_id00.Pos - zero_right_ID0) <= -(DOWN_LIMIT+TOLERANCE)) && zero_right_ID0 != 0)
        {STOP = True;}
    if (((MotorA1_recv_right_id01.Pos - zero_right_ID1) <= -(UP_LIMIT+TOLERANCE) || (MotorA1_recv_right_id01.Pos - zero_right_ID1) >= +(DOWN_LIMIT+TOLERANCE)) && zero_right_ID1 != 0)
        {STOP = True;}

    if(STOP==True)
      {HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12); 
       osDelay(300);
      } // 红灯闪烁
}
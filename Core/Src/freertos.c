/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_test.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "bsp_rc.h"
#include "OLED.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "MI_motor_drive.h"
#include "unitreeA1_cmd.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1415926535f
#define DGR2RAD PI/180
#define RAD2DGR 180/PI

#define UP 1
#define MID 3
#define DOWN 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

//定义全局变量
uint8_t STOP;  //急停按键
uint8_t STATE; //状态按键 (A1电机)
const RC_ctrl_t* DT7_pram; //遥控器控制结构体

MI_Motor_s MI_Motor_ID1;              // 定义小米电机结构体1
MI_Motor_s MI_Motor_ID2;              // 定义小米电机结构体2

extern motor_send_t cmd_left;         // 左腿一号电机数据体
extern motor_send_t cmd_right;        // 右腿一号电机数据体

extern motor_recv_t Date_left;        // 左腿电机接收数据体
extern motor_recv_t id00_left_date;   // 左腿00号电机接收数据体
extern motor_recv_t id01_left_date;   // 左腿01号电机接收数据体
extern motor_recv_t id02_left_date;   // 左腿02号电机接收数据体

// 默认电机零位
float zero_left_ID0 = 1000.0f;
float zero_left_ID1 = 1000.0f;
float zero_right_ID0 = 1000.0f;
float zero_right_ID1 = 1000.0f;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Motor_MIHandle;
osThreadId Motor_A1Handle;
osThreadId OLEDHandle;
osThreadId Motor_A1_TestHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Motor_MI_task(void const * argument);
void Motor_A1_task(void const * argument);
void OLED_task(void const * argument);
void Motor_A1_Test_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  //自定义 初始化 开始 ----------------------------------------------------------------
  HAL_Delay(1000);       // 延时1s 防止电机没上电先初始化现象
  int i = 0;
  OLED_init();           // OLED初始化
  OLED_clear();          OLED_printf(i/20,i%20,"#");  OLED_refresh_gram(); i++; // OLED清屏
  remote_control_init(); OLED_printf(i/20,i%20,"#");  OLED_refresh_gram(); i++; // 遥控器初始化
  
  CAN_Init(&hcan1);      OLED_printf(i/20,i%20,"#");  OLED_refresh_gram(); i++; // 初始化CAN1 + 打开中断FIFO0 FIFO1
  CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_0 | CAN_EXTID | CAN_DATA_TYPE, 0, 0); // 配置CAN1过滤器
  
  Buzzer_start(); // 蜂鸣器叫一声

  OLED_clear();

  //自定义 初始化 结束 ----------------------------------------------------------------

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Motor_MI */
  osThreadDef(Motor_MI, Motor_MI_task, osPriorityIdle, 0, 128);
  Motor_MIHandle = osThreadCreate(osThread(Motor_MI), NULL);

  /* definition and creation of Motor_A1 */
  osThreadDef(Motor_A1, Motor_A1_task, osPriorityIdle, 0, 128);
  Motor_A1Handle = osThreadCreate(osThread(Motor_A1), NULL);

  /* definition and creation of OLED */
  osThreadDef(OLED, OLED_task, osPriorityIdle, 0, 128);
  OLEDHandle = osThreadCreate(osThread(OLED), NULL);

  /* definition and creation of Motor_A1_Test */
  osThreadDef(Motor_A1_Test, Motor_A1_Test_task, osPriorityIdle, 0, 128);
  Motor_A1_TestHandle = osThreadCreate(osThread(Motor_A1_Test), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Motor_MI_task */
/**
* @brief Function implementing the Motor_MI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_MI_task */
void Motor_MI_task(void const * argument)
{
  /* USER CODE BEGIN Motor_MI_task */
  MI_motor_Init(&MI_Motor_ID1,&MI_CAN_1,1); // 将MI_CAN_1，ID=1传入小米结构体 
  MI_motor_Init(&MI_Motor_ID2,&MI_CAN_1,2); // 将MI_CAN_1，ID=2传入小米结构体 
  MI_motor_Enable(&MI_Motor_ID1);           // 通过发送小米结构体 data=00000000 电机使能
  MI_motor_Enable(&MI_Motor_ID2);           // 通过发送小米结构体 data=00000000 电机使能
  /* Infinite loop */
  for(;;)
  {
    // 小米电机控制
    MI_motor_SpeedControl(&MI_Motor_ID1,(float) STOP*DT7_pram->rc.ch[1]/33,1); // 使用 (float) 强制转换
    MI_motor_SpeedControl(&MI_Motor_ID2,(float) STOP*DT7_pram->rc.ch[3]/-33,1);

    // 小米电机模式
    // 力矩模式 MI_motor_TorqueControl()
    // 位置模式 MI_motor_LocationControl()
    // 速度模式 MI_motor_SpeedControl()

    osDelay(1);
  }
  /* USER CODE END Motor_MI_task */
}

/* USER CODE BEGIN Header_Motor_A1_task */
/**
* @brief Function implementing the Motor_A1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_A1_task */
void Motor_A1_task(void const * argument)
{
  /* USER CODE BEGIN Motor_A1_task */
  /*———————————————————————————————————左腿控制代码————————————————————————————————————————*/
  // 防止电机上电发疯
  STATE = UP;
  osDelay(100);

  // 电机零位 默认为1000，为了循环判断所以这么写 
  // 电机零位 定义在最上面
  // 使用while循环确保0位正确
  int continue_loop = 1; 
  while (continue_loop) {

      // 检查所有零位是否都在(-180, 180)范围内
      if ((zero_left_ID0 > -180 && zero_left_ID0 < 180) &&
          (zero_right_ID0 > -180 && zero_right_ID0 < 180) &&
          (zero_left_ID1 > -180 && zero_left_ID1 < 180) &&
          (zero_right_ID1 > -180 && zero_right_ID1 < 180)) {
          continue_loop = 0;  // 如果所有零位都在范围内，则结束循环
      }

      modfiy_torque_cmd(&cmd_left, 0, 0);    modfiy_torque_cmd(&cmd_right, 0, 0);
      unitreeA1_rxtx(&huart1);                unitreeA1_rxtx(&huart6);
      zero_left_ID0 = (float) id00_left_date.Pos * RAD2DGR / 9.1f;
      zero_right_ID0 = (float) id00_right_date.Pos * RAD2DGR / 9.1f;

      osDelay(5);

      modfiy_torque_cmd(&cmd_left, 1, 0);    modfiy_torque_cmd(&cmd_right, 1, 0);
      unitreeA1_rxtx(&huart1);                unitreeA1_rxtx(&huart6);
      zero_left_ID1 = (float) id01_left_date.Pos * RAD2DGR / 9.1f;
      zero_right_ID1 = (float) id01_right_date.Pos * RAD2DGR / 9.1f;

      osDelay(5);
  }


  /* Infinite loop */
  for(;;)
  {
    if (STATE == UP) // 急停 0力矩模式
    {
      modfiy_torque_cmd(&cmd_left,0,0);      modfiy_torque_cmd(&cmd_right,0,0);
      unitreeA1_rxtx(&huart1);               unitreeA1_rxtx(&huart6);
      osDelay(5);

      modfiy_torque_cmd(&cmd_left,1,0);      modfiy_torque_cmd(&cmd_right,1,0);
      unitreeA1_rxtx(&huart1);               unitreeA1_rxtx(&huart6);
      osDelay(5);
    }

    else if (STATE == MID)  // 速度模式
    {
      modfiy_speed_cmd(&cmd_left,0,(float) DT7_pram->rc.ch[2]/660*-30.0f);   modfiy_speed_cmd(&cmd_right,0,(float) DT7_pram->rc.ch[2]/660*30.0f);
      unitreeA1_rxtx(&huart1);                                               unitreeA1_rxtx(&huart6);
      osDelay(5);
      modfiy_speed_cmd(&cmd_left,1,(float) DT7_pram->rc.ch[0]/660*-30.0f);   modfiy_speed_cmd(&cmd_right,1,(float) DT7_pram->rc.ch[0]/660*30.0f);
      unitreeA1_rxtx(&huart1);                                               unitreeA1_rxtx(&huart6);
      osDelay(5);
    }

    else if (STATE == DOWN) // 位置模式
    {
      modfiy_cmd(&cmd_left,0,(float) DT7_pram->rc.ch[2]/660*-70 + zero_left_ID0, 0.005, 0.5);   
      modfiy_cmd(&cmd_right,0,(float) DT7_pram->rc.ch[2]/660*70 + zero_right_ID0, 0.005, 0.5); 
      unitreeA1_rxtx(&huart1); 
      unitreeA1_rxtx(&huart6);
      osDelay(5);
      modfiy_cmd(&cmd_left,1,(float) DT7_pram->rc.ch[0]/660*-70 + zero_left_ID1, 0.005, 0.5);   
      modfiy_cmd(&cmd_right,1,(float) DT7_pram->rc.ch[0]/660*70 + zero_right_ID0, 0.005, 0.5);
      unitreeA1_rxtx(&huart1);
      unitreeA1_rxtx(&huart6);
      osDelay(5);
    }
  }
  /* USER CODE END Motor_A1_task */
}

/* USER CODE BEGIN Header_OLED_task */
/**
* @brief Function implementing the OLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OLED_task */
void OLED_task(void const * argument)
{
  /* USER CODE BEGIN OLED_task */
  // OLED_show_string(0,0,"S1 = ");   OLED_show_string(0,10,"S0 = ");
  // OLED_show_string(1,0,"CH2= ");   OLED_show_string(1,10,"CH0= ");
  // OLED_show_string(2,0,"CH3= ");   OLED_show_string(2,10,"CH1= ");
  // OLED_show_string(3,0,"MI1= ");   OLED_show_string(3,10,"MI2= ");
  // OLED_show_string(4,0,"T1 =");    OLED_show_string(4,10,"T0 =");

  uint8_t i = 0;
  OLED_show_string(i,0,"ID0= ");   OLED_show_string(i,11,"ID1= "); i++;
  OLED_show_string(i,0,"T0 = ");   OLED_show_string(i,11,"T1 = "); i++;
  OLED_show_string(i,0,"P0 = ");   OLED_show_string(i,11,"P1 = "); i++;
  OLED_show_string(i,0,"W0 = ");   OLED_show_string(i,11,"W1 = "); i++;
  OLED_show_string(i,0,"A0 = ");   OLED_show_string(i,11,"A1 = "); i++;


  /* Infinite loop */
  for(;;)
  {
    // 任务 OLED + 遥控器接收
    DT7_pram = get_remote_control_point(); // 获取遥控器控制结构体
    STOP  = DT7_pram->rc.s[1]/2; // 跟踪遥控器开关 S[1]左 S[0]右 状态  // 上1 中3 下2
    STATE = DT7_pram->rc.s[0];   // 跟踪遥控器开关 S[1]左 S[0]右 状态  // 上1 中3 下2
                                           
    // 跟踪遥控器4个通道参数
    // OLED_show_num(0,5,(uint8_t) DT7_pram->rc.s[1]/2,1);       OLED_show_num(0,15,(uint8_t) DT7_pram->rc.s[0]/2,1);
    // OLED_show_signednum(1,5,DT7_pram->rc.ch[2],3);            OLED_show_signednum(1,15,DT7_pram->rc.ch[0],3);
    // OLED_show_signednum(2,5,DT7_pram->rc.ch[3],3);            OLED_show_signednum(2,15,DT7_pram->rc.ch[1],3);
    // OLED_show_signednum(3,5,MI_Motor_ID1.RxCAN_info.speed,3); OLED_show_signednum(3,15,MI_Motor_ID2.RxCAN_info.speed,3);
    // OLED_show_signednum(4,5,id01_left_date.T,3);              OLED_show_signednum(4,15,id00_left_date.T,3);

    uint8_t i = 0;
    OLED_show_signednum(i,5,id00_left_date.motor_id,3);                OLED_show_signednum(i,16,id01_left_date.motor_id,3);          i++;
    OLED_show_signednum(i,5,id00_left_date.T*100,3);                   OLED_show_signednum(i,16,id01_left_date.T*100,3);             i++;
    OLED_show_signednum(i,5,id00_left_date.Pos*RAD2DGR/9.1f,4);        OLED_show_signednum(i,16,id01_left_date.Pos*RAD2DGR/9.1f,4);  i++;
    OLED_show_signednum(i,5,id00_left_date.W,3);                       OLED_show_signednum(i,16,id01_left_date.W,3);                 i++;
    // OLED_show_signednum(i,5,id00_left_date.Acc/9.1f,3);                OLED_show_signednum(i,16,id01_left_date.Acc/9.1f,3);          i++;
    OLED_show_signednum(i,5,id00_left_date.Acc/9.1f,3);                OLED_show_signednum(i,16,id01_left_date.Acc/9.1f,3);          i++;
    OLED_refresh_gram();
    osDelay(5);
  }
  /* USER CODE END OLED_task */
}

/* USER CODE BEGIN Header_Motor_A1_Test_task */
/**
* @brief Function implementing the Motor_A1_Test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_A1_Test_task */
void Motor_A1_Test_task(void const * argument)
{
  /* USER CODE BEGIN Motor_A1_Test_task */

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Motor_A1_Test_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include <string.h>
#include "INS_task.h"
#include "OLED.h"
#include "buzzer.h"
#include "rc.h"
#include "bsp_usart.h"
#include "bsp_rc.h"
#include "bsp_delay.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "can.h"
#include "can_test.h"
#include "MI_motor_drive.h"
#include "unitreeA1_cmd.h"
#include "joint.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1415926f
#define DRG 180/PI

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//定义全局变量
uint8_t MODE;  // 模式 (SW_UP=关闭 SW_MID=遥控模式 SW_DOWN=电脑模式)
uint8_t STATE; // 状态 (A1电机)

extern RC_Type rc;        // 遥控器数据

extern fp32 INS_angle[3]; // 陀螺仪角度
extern fp32 temp;         // BMI088温度

MI_Motor_s MI_Motor_ID1;              // 定义小米电机结构体1
MI_Motor_s MI_Motor_ID2;              // 定义小米电机结构体2

extern motor_send_t cmd_left;         // 左腿一号电机数据体
extern motor_send_t cmd_right;        // 右腿一号电机数据体

extern motor_recv_t Date_left;        // 左腿电机接收数据体
extern motor_recv_t id00_left_date;   // 左腿00号电机接收数据体
extern motor_recv_t id01_left_date;   // 左腿01号电机接收数据体
extern motor_recv_t id02_left_date;   // 左腿02号电机接收数据体

// 默认电机零位
extern float zero_left_ID0;
extern float zero_left_ID1;
extern float zero_right_ID0;
extern float zero_right_ID1;

float POS_BUF = 0.0f; 

/* USER CODE END Variables */
osThreadId testHandle;
osThreadId OLEDHandle;
osThreadId Motor_MIHandle;
osThreadId Motor_A1Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);
void OLED_task(void const * argument);
void Motor_MI_task(void const * argument);
void Motor_A1_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  uint8_t i=0;
  OLED_init();
  Buzzer_beep();
  // HAL_Delay(500);  // 不能用HAL_Delay
  osDelay(500); // 延时初始化 CAN， 防止初始化早于上电 (需要写CAN自检程序)
  delay_init();          OLED_printf(i/20,i%20,"#");  OLED_refresh_gram(); i++; // 与BMI088_init()相关
  Dbus_Init();           OLED_printf(i/20,i%20,"#");  OLED_refresh_gram(); i++; // 遥控器初始化
  CAN_Init(&hcan1);      OLED_printf(i/20,i%20,"#");  OLED_refresh_gram(); i++; // 初始化CAN1 + 打开中断FIFO0 FIFO1
  CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_0 | CAN_EXTID | CAN_DATA_TYPE, 0, 0); // 配置CAN1过滤器

  OLED_clear();
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET); 

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
  /* definition and creation of test */
  osThreadDef(test, test_task, osPriorityNormal, 0, 128);
  testHandle = osThreadCreate(osThread(test), NULL);

  /* definition and creation of OLED */
  osThreadDef(OLED, OLED_task, osPriorityIdle, 0, 128);
  OLEDHandle = osThreadCreate(osThread(OLED), NULL);

  /* definition and creation of Motor_MI */
  osThreadDef(Motor_MI, Motor_MI_task, osPriorityIdle, 0, 128);
  Motor_MIHandle = osThreadCreate(osThread(Motor_MI), NULL);

  /* definition and creation of Motor_A1 */
  osThreadDef(Motor_A1, Motor_A1_task, osPriorityIdle, 0, 128);
  Motor_A1Handle = osThreadCreate(osThread(Motor_A1), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  // 这里是官方手写的 FreeRTOS 线程，.ioc不显示这些任务
    osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* USER CODE BEGIN test_task */

  /* Infinite loop */
  for(;;)
  {
    if (POS_BUF > -80.0 && POS_BUF < 20.0)
      {POS_BUF = POS_BUF + rc.RY;}

    else if (POS_BUF <= -80.0)
      {POS_BUF = -79.0;}

    else if (POS_BUF >= 20.0)
      {POS_BUF = 19.0;}

    osDelay(10);
  }
  /* USER CODE END test_task */
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
  uint8_t i = 0;
  OLED_show_string(i,0,"Yaw   = "); i++;
  OLED_show_string(i,0,"Pitch = "); i++;
  OLED_show_string(i,0,"Roll  = "); i++;
  OLED_show_string(i,0,"BUF = ");   
  OLED_refresh_gram();
  /* Infinite loop */
  for(;;)
  {
    // 任务 OLED + 遥控器接收
    MODE  = rc.sw1/2; // 跟踪遥控器开关 S[1]左 S[0]右 状态  // 上1 中3 下2
    STATE = rc.sw2;   // 跟踪遥控器开关 S[1]左 S[0]右 状态  // 上1 中3 下2

    i = 0;
    OLED_show_signednum(i,9,INS_angle[0]*DRG,3);   i++;
    OLED_show_signednum(i,9,INS_angle[1]*DRG,3);   i++;
    OLED_show_signednum(i,9,INS_angle[2]*DRG,3);   i++;
    OLED_show_signednum(i,7,POS_BUF,3);    
    OLED_refresh_gram();
    osDelay(2);
  }
  /* USER CODE END OLED_task */
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
    MI_motor_SpeedControl(&MI_Motor_ID1, MODE*(+1)*(rc.LY-rc.RX)*20,1);  // 左轮
    MI_motor_SpeedControl(&MI_Motor_ID2, MODE*(-1)*(rc.LY+rc.RX)*20,1);  // 右轮
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
  // 防止电机上电发疯
  STATE = SW_UP;
  osDelay(100);

  // 电机零位初始化
  Joint_Zero_init_Type1();

  /* Infinite loop */
  for(;;)
  {
    if (STATE == SW_UP) // 急停 0力矩模式
    {
      modfiy_torque_cmd(&cmd_left,0,0);      modfiy_torque_cmd(&cmd_right,0,0);
      unitreeA1_rxtx(&huart1);               unitreeA1_rxtx(&huart6);
      osDelay(2);

      modfiy_torque_cmd(&cmd_left,1,0);      modfiy_torque_cmd(&cmd_right,1,0);
      unitreeA1_rxtx(&huart1);               unitreeA1_rxtx(&huart6);
      osDelay(2);
    }

    else if (STATE == SW_MID)  // 速度模式
    {
      modfiy_speed_cmd(&cmd_left,0,(float) rc.RX*30.0f);   modfiy_speed_cmd(&cmd_right,0,(float) rc.RX*-30.0f);
      unitreeA1_rxtx(&huart1);                                               unitreeA1_rxtx(&huart6);
      osDelay(2);
      modfiy_speed_cmd(&cmd_left,1,(float) rc.LX*30.0f);   modfiy_speed_cmd(&cmd_right,1,(float) rc.LX*-30.0f);
      unitreeA1_rxtx(&huart1);                                               unitreeA1_rxtx(&huart6);
      osDelay(2);
    }

    else if (STATE == SW_DOWN) // 位置模式 (现在的位置模式为减速后的转子角度-角度制)
    {
      modfiy_cmd(&cmd_left,0,(float) rc.RX*70 + zero_left_ID0, 0.006, 1.0);  // 0.005 0.5  
      modfiy_cmd(&cmd_right,0,(float) rc.RX*-70 + zero_right_ID0, 0.006,1.0); 
      unitreeA1_rxtx(&huart1); 
      unitreeA1_rxtx(&huart6);
      osDelay(2);
      modfiy_cmd(&cmd_left,1,(float) rc.LX*70 + zero_left_ID1, 0.006, 1.0);   
      modfiy_cmd(&cmd_right,1,(float) rc.LX*-70 + zero_right_ID1, 0.006, 1.0);
      unitreeA1_rxtx(&huart1);
      unitreeA1_rxtx(&huart6);
      osDelay(2);
    }
  }
  /* USER CODE END Motor_A1_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

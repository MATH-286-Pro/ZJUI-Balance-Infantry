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
#include "bsp_usart.h"
#include "bsp_rc.h"
#include "bsp_delay.h"
#include "INS_task.h"
#include "OLED.h"
#include "buzzer.h"
#include "rc.h"
#include "can_test.h"
#include "MI_motor_drive.h"
#include "A1_motor_drive.h"
#include "joint.h"
#include "wheel.h"
#include "ZJUI_balance.h"
#include "pid.h"


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
extern RC_Type rc;        // 遥控器数据

extern MI_Motor_s MI_Motor_ID1;               // 定义小米电机结构体1
extern MI_Motor_s MI_Motor_ID2;               // 定义小米电机结构体2

extern motor_send_t MotorA1_send_left;        // 左腿一号电机数据体
extern motor_send_t MotorA1_send_right;       // 右腿一号电机数据体

extern motor_recv_t Date_left;                // 左腿电机接收数据体
extern motor_recv_t MotorA1_recv_left_id00;   // 左腿00号电机接收数据体
extern motor_recv_t MotorA1_recv_left_id01;   // 左腿01号电机接收数据体

extern motor_recv_t Date_right;               // 右腿电机接收数据体
extern motor_recv_t MotorA1_recv_right_id00;  // 右腿00号电机接收数据体
extern motor_recv_t MotorA1_recv_right_id01;  // 右腿01号电机接收数据体

// 默认电机零位 角度制 减速后的角度
extern float zero_left_ID0;
extern float zero_left_ID1;
extern float zero_right_ID0;
extern float zero_right_ID1;

extern uint8_t STOP; // 急停状态

// 平衡 PID 参数
extern pid_type_def PID_L; 
extern pid_type_def PID_R; 
extern pid_type_def PID_VEL;
extern Vel_measure;
extern Vel_measure_mod;
extern pid_type_def PID_VEL_UP;   // 速度环 PID 结构体 (±10°都非常稳定)

// 测试参数
float BUF = 0.0f; // 用于轮腿升降测试

/* USER CODE END Variables */
osThreadId testHandle;
osThreadId OLEDHandle;
osThreadId Motor_MIHandle;
osThreadId Motor_A1Handle;
osThreadId RobotHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);
void OLED_task(void const * argument);
void Motor_MI_task(void const * argument);
void Motor_A1_task(void const * argument);
void Robot_task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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
  osThreadDef(OLED, OLED_task, osPriorityIdle, 0, 512);
  OLEDHandle = osThreadCreate(osThread(OLED), NULL);

  /* definition and creation of Motor_MI */
  osThreadDef(Motor_MI, Motor_MI_task, osPriorityIdle, 0, 128);
  Motor_MIHandle = osThreadCreate(osThread(Motor_MI), NULL);

  /* definition and creation of Motor_A1 */
  osThreadDef(Motor_A1, Motor_A1_task, osPriorityIdle, 0, 128);
  Motor_A1Handle = osThreadCreate(osThread(Motor_A1), NULL);

  /* definition and creation of Robot */
  osThreadDef(Robot, Robot_task, osPriorityIdle, 0, 512);
  RobotHandle = osThreadCreate(osThread(Robot), NULL);

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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN test_task */

  /* Infinite loop */
  for(;;)
  {
    // if (rc.sw1 == SW_MID)
    //   {STOP = False;
    //   HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET);} // 复位
    // Joint_Monitor();
    // USB_printf("Output:%d\n",(int)(INS.Pitch*DRG*100));  // Hard falut
    osDelay(100);
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
  // uint8_t i = 0;
  // OLED_show_string(i,0,"Yaw=");   OLED_show_string(i,10,"Rol=");  i++;
  // OLED_show_string(i,0,"Pit=");   i++;
  // OLED_refresh_gram();
  float speed_print = 0.0f;
  float speed_print_last = 0.0f;
  /* Infinite loop */
  for(;;)
  {
    // 任务 OLED + 遥控器接收
    // i = 0;
    // OLED_show_signednum(i,4,INS_angle[0]*DRG,3);    OLED_show_signednum(i,10+4,INS_angle[2]*DRG,3);   i++;
    // OLED_show_signednum(i,4,INS_angle[1]*DRG,3);    i++;
    // OLED_refresh_gram();
    speed_print_last = speed_print;
    speed_print = (-MI_Motor_ID2.RxCAN_info.speed + MI_Motor_ID1.RxCAN_info.speed)/2*62*0.001;
    speed_print = speed_print*0.3 + speed_print_last*0.7;
    USB_printf("Output:%d,%d,%d,%d\n", (int)(INS.Pitch*DRG*100),(int)(speed_print*1000),
                                       (int)(rc.LY*2.0f*1000),(int)(PID_VEL_UP.out*1000));

    osDelay(10);
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
  // MI_motor_Init(&MI_Motor_ID1,&MI_CAN_1,1); // 将MI_CAN_1，ID=1传入小米结构体 
  // MI_motor_Init(&MI_Motor_ID2,&MI_CAN_1,2); // 将MI_CAN_1，ID=2传入小米结构体 
  // MI_motor_Enable(&MI_Motor_ID1);           // 通过发送小米结构体 data=00000000 电机使能
  // MI_motor_Enable(&MI_Motor_ID2);           // 通过发送小米结构体 data=00000000 电机使能
  // osDelay(100);

  /* Infinite loop */
  for(;;)
  {
    // if (rc.sw1 == SW_UP){Wheel_Speed_Control(rc.LY*20 + rc.RX*20, rc.LY*20 - rc.RX*20);} // 轮速度控制，正值前进，负值后退
    // if (rc.sw1 == SW_MID){Wheel_Speed_Control(0.0f,0.0f);}          // 轮电机停转
    osDelay(10);
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
  osDelay(100);

  // 电机零位初始化
  // Joint_Zero_init_Type1(); // 上电原点
  Joint_Zero_init_Type2(); // 限位原点
  osDelay(10);
  Joint_Speed_Control(0.0f,0.0f);
  // Joint_Position_Control(0.0f,0.0f);


  /* Infinite loop */
  for(;;)
  {
    if (rc.sw2 == SW_UP || rc.sw2 == SW_POWER_OFF || STOP == True) // 急停
    {
      // Joint_Speed_Control(rc.RX*3.0f,-rc.LX*3.0f);
      Joint_Speed_Control(0.0f,0.0f);
    }

    else if (rc.sw2 == SW_MID && STOP == False) // 位置模式 (现在的位置模式为减速后的转子角度-角度制)
    {
      //                       RX向右为正    LX向左为负
      // Joint_Position_Control(rc.RX*70.0f,-rc.LX*70.0f);
      // Joint_Position_Control(0.0f,0.0f);
      Joint_Position_Control(rc.LY*60.0f,rc.LY*60.0f);
    }
  }
  /* USER CODE END Motor_A1_task */
}

/* USER CODE BEGIN Header_Robot_task */
/**
* @brief Function implementing the Robot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Robot_task */
void Robot_task(void const * argument)
{
  /* USER CODE BEGIN Robot_task */
  MI_motor_Init(&MI_Motor_ID1,&MI_CAN_1,1); // 将MI_CAN_1，ID=1传入小米结构体 
  MI_motor_Init(&MI_Motor_ID2,&MI_CAN_1,2); // 将MI_CAN_1，ID=2传入小米结构体 
  MI_motor_Enable(&MI_Motor_ID1);           // 通过发送小米结构体 data=00000000 电机使能
  MI_motor_Enable(&MI_Motor_ID2);           // 通过发送小米结构体 data=00000000 电机使能
  osDelay(100);
  /* Infinite loop */
  for(;;)
  {
    if (rc.sw1 == SW_UP && STOP == False)   //倒地-遥控模式
    {
      Wheel_Speed_Control(rc.RY*20 + rc.RX*20, rc.RY*20 - rc.RX*20);
      osDelay(2);
    }
    if (rc.sw1 == SW_MID && STOP == False)  //平衡-遥控模式
    {
      stand_task_start(&INS, rc.RY, rc.RX);
      osDelay(2);
    }
    if (rc.sw1 == SW_DOWN && STOP == False) //平衡-跟踪模式
    {
      stand_task_start(&INS, rc.RY, rc.RX);
      osDelay(2);
    }
  }
  /* USER CODE END Robot_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

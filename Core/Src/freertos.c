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
#include "can.h"
#include "OLED.h"
#include "Debug_Tool.h"
#include "ins_task.h"
#include "ZJUI_balance.h"
#include "bsp_dwt.h"
#include "usbd_cdc_if.h"
#include "rc.h"
#include "bsp_buzzer.h"
#include "MI_motor_drive.h"
#include "A1_motor_drive.h"
#include "joint.h"
#include "wheel.h"
#include "Balance.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern LinkNPodParam l_side, r_side;    
extern ChassisParam chassis;

extern MI_Motor_s MI_Motor_ID1;               // 定义小米电机结构体1
extern MI_Motor_s MI_Motor_ID2;               // 定义小米电机结构体2

extern RC_Type rc;        // 遥控器数据

extern INS_t *INS_DATA;
extern pid_type_def PID_Balance;
extern pid_type_def PID_VEL_UP;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId insTaskHandle;   // 手动添加
osThreadId robotTaskHandle; // 手动添加
osThreadId Motor_A1Handle;  // 手动添加


/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartINSTASK(void const *argument);   // 手动添加
void StartROBOTTASK(void const *argument); // 手动添加
void Motor_A1_task(void const * argument); // 手动添加


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512); // 原先为 128
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(instask, StartINSTASK, osPriorityNormal, 0, 1024); // 手动添加
  insTaskHandle = osThreadCreate(osThread(instask), NULL);

  osThreadDef(robottask, StartROBOTTASK, osPriorityAboveNormal, 0, 2048); // 手动添加
  robotTaskHandle = osThreadCreate(osThread(robottask), NULL);

  osThreadDef(Motor_A1, Motor_A1_task, osPriorityIdle, 0, 128);
  Motor_A1Handle = osThreadCreate(osThread(Motor_A1), NULL);
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
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  for(;;)
  {
    // USB_printf("Velocity.Dist.motionN:%d,%d,%d,%d,%d,%d,%d,%d,%d\n",(int)(INS_DATA->Yaw),(int)(INS_DATA->Pitch),(int)(INS_DATA->Roll),
    //                                     (int)(INS_DATA->MotionAccel_b[0]*100),(int)(INS_DATA->MotionAccel_b[1]*100),(int)(INS_DATA->MotionAccel_b[2]*100),
    //                                     (int)(INS_DATA->Vel[0]*100),(int)(INS_DATA->Vel[1]*100),(int)(INS_DATA->Vel[2]*100));
    // USB_printf("Velocity.Dist.motionN:%d,%d\n",(int)(chassis.dist*100),(int)(chassis.vel_m*100));

    // 湖南大学 Pitch Roll 需要互换 
    USB_printf("Yaw.Roll.Pitch:%d,%d,%d,%d,%d,%d\n",(int)(INS_DATA->Yaw),(int)(INS_DATA->Pitch),
                                              (int)(INS_DATA->Roll),(int)(INS_DATA->Gyro[Y]*180/3.14),
                                              (int)(PID_VEL_UP.out*100)),(int)(PID_Balance.out*100);
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartINSTASK(void const *argument)
{
  while (1)
  {
    // 1kHz
    INS_Task(); 
    osDelay(1);
  }
}

void Motor_A1_task(void const * argument)
{
  /* USER CODE BEGIN Motor_A1_task */
  osDelay(100);
  Joint_Zero_init_Type1(); // 上电原点
  osDelay(10);
  Joint_Speed_Control(0.0f,0.0f);
  /* Infinite loop */
  for(;;)
  {
    if (rc.sw1 == SW_UP) // 急停
    {
      // Joint_Speed_Control(rc.RX*3.0f,-rc.LX*3.0f);
      Joint_Position_Control(0.0f,0.0f);
    }

    else if (rc.sw1 == SW_MID) // 位置模式 (现在的位置模式为减速后的转子角度-角度制)
    {
      Joint_Position_Control(0.0f,0.0f);
      // Joint_Full_Position_Control(rc.LY*60.0f - rc.LX*20.0f,
      //                             rc.LY*60.0f + rc.LX*20.0f,
      //                             rc.LY*60.0f - rc.LX*20.0f,
      //                             rc.LY*60.0f + rc.LX*20.0f);
    }
  }
  /* USER CODE END Motor_A1_task */
}

void StartROBOTTASK(void const *argument)
{
  // static float robot_dt, robot_start;
  MI_motor_Init(&MI_Motor_ID1,&MI_CAN_1,1); // 将MI_CAN_1，ID=1传入小米结构体 
  MI_motor_Init(&MI_Motor_ID2,&MI_CAN_1,2); // 将MI_CAN_1，ID=2传入小米结构体 
  MI_motor_Enable(&MI_Motor_ID1);           // 通过发送小米结构体 data=00000000 电机使能
  MI_motor_Enable(&MI_Motor_ID2);           // 通过发送小米结构体 data=00000000 电机使能
  osDelay(100);
  for(;;)
  {
    // robot_start = DWT_GetTimeline_ms();
    // BalanceTask();
    // robot_dt = DWT_GetTimeline_ms() - robot_start;
    // osDelay(1);
    if (rc.sw2 == SW_UP)   //倒地-遥控模式
    {
      Wheel_Speed_Control(rc.RY*20 + rc.RX*20, rc.RY*20 - rc.RX*20);
      osDelay(2);
    }
    if (rc.sw2 == SW_MID)  //平衡-遥控模式
    {
      stand_task_start(INS_DATA, rc.RY, rc.RX);
      osDelay(2);
    }
    if (rc.sw2 == SW_DOWN) //平衡-跟踪模式
    {
      stand_task_start(INS_DATA, rc.RY, rc.RX);
      osDelay(2);
    }
  }
}
/* USER CODE END Application */



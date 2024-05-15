/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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
#include "usbd_cdc_if.h"
#include "ins_task.h"

/* USER CODE BEGIN INCLUDE */
#define RC_RATIO 660.0f  // 摇杆数值CH归一化比例
#define RC_OFFSET 1024   // 摇杆数值CH的偏移量
#define PI 3.1415926f
#define DRG 180/PI


static USBCallback tx_cbk = NULL;
static USBCallback rx_cbk = NULL;

// extern fp32 INS_angle[3]; // 陀螺仪角度
extern INS_t *INS_DATA;
uint32_t usb_rc_tick;                       // 接收到数据的时刻
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */
RC_ctrl_t rc_ctrl;

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}



void rc_init(RC_ctrl_t *rc)
{
    //  得到摇杆数据
    // 根据摇杆数据的偏置和比例解析出[-1, 1]之间的数值
    rc->ch1 = 0;
    rc->RX = 0;
    rc->ch2 = 0;
    rc->RY = 0;
    rc->ch3 = 0;
    rc->LX = 0;
    rc->ch4 = 0;
    rc->LY = 0;

    return;
}


/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */


/* USER CODE BEGIN EXPORTED_VARIABLES */
USBD_CDC_LineCodingTypeDef USBD_CDC_LineCoding = 
{
  115200,
  0x00,
  0x00,
  0x08,
};
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
static void sbus_to_rc(volatile uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

static void sbus_to_rc(volatile uint8_t *buff, RC_ctrl_t *rc)
{
    
    usb_rc_tick = HAL_GetTick();                                                                           // 获取系统运行时间
    if (buff[0] == 0 && buff[1] == 0 && buff[2] == 0 && buff[3] == 0 && buff[4] == 0 && buff[5] == 0)  // 不知道这一段是为了什么，先保留了
    {
      return;
    }

    //  得到摇杆数据
    rc->ch1 = (buff[0] | (buff[1] << 8)) & 0x07FF;
    rc->ch2 = ((buff[1] >> 3) | (buff[2] << 5)) & 0x07FF;
    rc->ch3 = ((buff[2] >> 6) | (buff[3] << 2) | (buff[4] << 10)) & 0x07FF;
    rc->ch4 = ((buff[4] >> 1) | (buff[5] << 7)) & 0x07FF;
    // 根据摇杆数据的偏置和比例解析出[-1, 1]之间的数值
    rc->ch1 -= RC_OFFSET;
    rc->RX = rc->ch1 / RC_RATIO;
    
    rc->RY = rc->ch2 ;
    
    rc->LX = rc->ch3 ;
    rc->ch4 -= RC_OFFSET;
    rc->LY = rc->ch4 / RC_RATIO;

    return;
}
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */

  // USBD_CDC_LineCodingTypeDef USBD_CDC_LineCoding; // 添加测试

  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:
      USBD_CDC_LineCoding.bitrate = (pbuf[3]<<24) | (pbuf[2]<<16) | (pbuf[1]<<8) | pbuf[0];
      USBD_CDC_LineCoding.format = pbuf[4];
      USBD_CDC_LineCoding.paritytype = pbuf[5];
      USBD_CDC_LineCoding.datatype = pbuf[6];
    break;

    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)(USBD_CDC_LineCoding.bitrate);
      pbuf[1] = (uint8_t)(USBD_CDC_LineCoding.bitrate >> 8);
      pbuf[2] = (uint8_t)(USBD_CDC_LineCoding.bitrate >> 16);
      pbuf[3] = (uint8_t)(USBD_CDC_LineCoding.bitrate >> 24);
      pbuf[4] = USBD_CDC_LineCoding.format;
      pbuf[5] = USBD_CDC_LineCoding.paritytype;
      pbuf[6] = USBD_CDC_LineCoding.datatype;

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  uint8_t result = USBD_OK;
  uint8_t output_buf[4];  // 为了确保能够容纳所有的字节，数组大小设置为5

  sbus_to_rc(Buf, &rc_ctrl);

  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  // 将uint16_t的地址转换为uint8_t的指针
  // USB_printf("Output:%d,%d\n", (int)(PID_STANDE.out),(int)(INS.Pitch*DRG*100));
  // int ori = (int)(INS_angle[0]*DRG*100) + 3600;
  int ori = (int)(INS_DATA->Yaw)*100 + 3600;

  output_buf[0] = (ori >> 24) & 0xFF; // 最高字节
  output_buf[1] = (ori >> 16) & 0xFF;
  output_buf[2] = (ori >> 8) & 0xFF;
  output_buf[3] = ori & 0xFF;          // 最低字节

  result = CDC_Transmit_FS(output_buf, sizeof(output_buf));

  
  // 用更新后的缓冲区和长度发送数据
  // CDC_Transmit_FS(output_buf, sizeof(output_buf));

  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  if(tx_cbk)
    tx_cbk(*Len);
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
uint8_t* CDCInitRxbufferNcallback(USBCallback transmit_cbk,USBCallback recv_cbk)
{
  tx_cbk = transmit_cbk;
  rx_cbk = recv_cbk;
  return UserRxBufferFS;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

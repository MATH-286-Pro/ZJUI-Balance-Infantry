#ifndef __CAN_TEST_H__
#define __CAN_TEST_H__


// 滤波器编号
#define CAN_FILTER(x) ((x) << 3) //位运算

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

// 标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)


void CAN_Init(CAN_HandleTypeDef *hcan);

uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);

void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

void LED_Control(uint8_t data);

#endif


#include "main.h"
#include "can.h"
#include "Callback.h"
#include "MI_motor_drive.h"
#include "freeRTOS.h"

/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    RxCAN_info_s RxCAN_info;//用于存储小米电机反馈的数据
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    memcpy(&RxCAN_info,&rx_header.ExtId,4);//将扩展标识符的内容解码到缓存区，获取通信类型

    if(RxCAN_info.communication_type == 0){//通信类型0的反馈帧解码
        RxCAN_info_type_0_s RxCAN_info_type_0;
        memcpy(&RxCAN_info_type_0,&rx_header.ExtId,4);//将扩展标识符的内容解码成通信类型0的对应内容
        memcpy(&RxCAN_info_type_0.MCU_id,rx_data,8);//获取MCU标识符
        //OutputData.data_3 = RxCAN_info_type_0.motor_id;
    }else if(RxCAN_info.communication_type == 2){//通信类型2的反馈帧解码
        RxCAN_info_type_2_s RxCAN_info_type_2;
        memcpy(&RxCAN_info_type_2,&rx_header.ExtId,4);//将扩展标识符的内容解码成通信类型2的对应内容
        MI_motor_RxDecode(&RxCAN_info_type_2,rx_data);//通信类型2的数据解码
        if (RxCAN_info_type_2.motor_id == 1){
            // 注意设置 MI_Motor_ID
            MI_Motor_ID1.RxCAN_info = RxCAN_info_type_2;
            MI_Motor_ID1.motor_mode_state = RxCAN_info_type_2.mode_state;
        }
        else if (RxCAN_info_type_2.motor_id == 2){
            // 注意设置 MI_Motor_ID
            MI_Motor_ID2.RxCAN_info = RxCAN_info_type_2;
            MI_Motor_ID2.motor_mode_state = RxCAN_info_type_2.mode_state;
        }
    }else if(RxCAN_info.communication_type == 17){//通信类型17的反馈帧解码
        RxCAN_info_type_17_s RxCAN_info_type_17;
        memcpy(&RxCAN_info_type_17,&rx_header.ExtId,4);//将扩展标识符的内容解码成通信类型17的对应内容
        memcpy(&RxCAN_info_type_17.index,&rx_data[0],2);//获取查找的参数索引码
        memcpy(&RxCAN_info_type_17.param,&rx_data[4],4);//获取查找的参数信息
    }
}

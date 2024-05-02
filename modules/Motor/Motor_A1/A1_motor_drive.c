#include "main.h"
#include "motor_msg.h"
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "A1_motor_drive.h"

#define PI 3.14159

motor_send_t MotorA1_send_left;       // 左腿一号电机数据体
motor_send_t MotorA1_send_right;      // 右腿一号电机数据体

motor_recv_t Date_left;               // 左腿电机接收数据体
motor_recv_t MotorA1_recv_left_id00;  // 左腿00号电机接收数据体
motor_recv_t MotorA1_recv_left_id01;  // 左腿01号电机接收数据体
motor_recv_t MotorA1_recv_left_id02;  // 左腿02号电机接收数据体

motor_recv_t Date_right;              // 右腿电机接收数据体
motor_recv_t MotorA1_recv_right_id00; // 右腿00号电机接收数据体
motor_recv_t MotorA1_recv_right_id01; // 右腿01号电机接收数据体
motor_recv_t MotorA1_recv_right_id02; // 右腿02号电机接收数据体

// CRC校验位的代码
uint32_t crc32_core_Ver3(uint32_t *ptr, uint32_t len)
{
    uint32_t bits;
    uint32_t i;
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}

// 电机位置修改
void modfiy_pos_cmd(motor_send_t *send,uint8_t id, float Pos, float KP, float KW)
{

    send->hex_len = 34;

    send->mode = 10;
	send->id   = id;

    send->Pos  = 2*PI/360*9.1*Pos;  // 6.2832 = 2 PI // 原先为 6.2832*9.1*2*Pos
    send->W    = 0;
    send->T    = 0.0;
    send->K_P  = KP;
    send->K_W  = KW;
}

// 电机速度修改
void modfiy_speed_cmd(motor_send_t *send,uint8_t id, float Omega)
{

    send->hex_len = 34;

    send->mode = 10;
	send->id   = id;

    send->Pos  = 0;
    send->W    = Omega * 9.1f;
    send->T    = 0.0;
    send->K_P  = 0.0;
    send->K_W  = 3.0;
}

// 电机力矩修改
void modfiy_torque_cmd(motor_send_t *send,uint8_t id, float torque)
{

    send->hex_len = 34;

    send->mode = 10;
	send->id   = id;

    send->Pos  = 0.0;
    send->W    = 0.0;
    if (torque > 10.0f){torque = 0.0f;} // 限幅
    send->T    = torque / 9.1f;
    send->K_P  = 0.0;
    send->K_W  = 0.0;
}

// 电机发送接收函数
void unitreeA1_rxtx(UART_HandleTypeDef *huart)
{
    /*—————————————————————————————————————————左腿代码范围————————————————————————————————————————————————*/
    if (huart == &huart1)
    {
        uint8_t A1MotorA1_send_left[34]; // 发送数据
        uint8_t Date[78];       // 接收数据

        // 此处为左腿电机结构体//
        MotorA1_send_left.motor_send_data.head.start[0] = 0xFE;
        MotorA1_send_left.motor_send_data.head.start[1] = 0xEE;
        MotorA1_send_left.motor_send_data.head.motorID  = MotorA1_send_left.id;
        MotorA1_send_left.motor_send_data.head.reserved = 0x00;

        MotorA1_send_left.motor_send_data.Mdata.mode      = MotorA1_send_left.mode;  // mode = 10
        MotorA1_send_left.motor_send_data.Mdata.ModifyBit = 0xFF;
        MotorA1_send_left.motor_send_data.Mdata.ReadBit   = 0x00;
        MotorA1_send_left.motor_send_data.Mdata.reserved  = 0x00;
        MotorA1_send_left.motor_send_data.Mdata.Modify.F  = 0;
        MotorA1_send_left.motor_send_data.Mdata.T         = MotorA1_send_left.T * 256;
        MotorA1_send_left.motor_send_data.Mdata.W         = MotorA1_send_left.W * 128;
        MotorA1_send_left.motor_send_data.Mdata.Pos       = (int)((MotorA1_send_left.Pos / 6.2832f) * 16384.0f); // 单位 rad
        MotorA1_send_left.motor_send_data.Mdata.K_P       = MotorA1_send_left.K_P * 2048;
        MotorA1_send_left.motor_send_data.Mdata.K_W       = MotorA1_send_left.K_W * 1024;

        MotorA1_send_left.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
        MotorA1_send_left.motor_send_data.Mdata.LowHzMotorCmdByte  = 0;
        MotorA1_send_left.motor_send_data.Mdata.Res[0] = MotorA1_send_left.Res;

        MotorA1_send_left.motor_send_data.CRCdata.u32 = crc32_core_Ver3((uint32_t *)(&MotorA1_send_left.motor_send_data), 7); // CRC校验

        memcpy(A1MotorA1_send_left, &MotorA1_send_left.motor_send_data, 34);
        
        // HAL库 DMA 发送数据 + 接收数据
        HAL_UART_Transmit_DMA(&huart1, A1MotorA1_send_left, 34);
        HAL_Delay(10);
        HAL_UART_Receive_DMA(&huart1, Date, 78);

        // 接受数据处理
        // 1.没有处理温度数据 (可能正确，因为是整数?)
        // 2.检查数据类型是否都正确
        Date_left.motor_recv_data.head.motorID = Date[2];  
        Date_left.motor_recv_data.Mdata.mode   = Date[4];  
        Date_left.motor_recv_data.Mdata.Temp   = Date[6];
        Date_left.motor_recv_data.Mdata.MError = Date[7]; 
        Date_left.motor_recv_data.Mdata.T      = Date[13] << 8  | Date[12]; // 反拼
        Date_left.motor_recv_data.Mdata.W      = Date[15] << 8  | Date[14]; // 反拼
        Date_left.motor_recv_data.Mdata.Acc    = Date[27] << 8  | Date[26]; // 反拼
        Date_left.motor_recv_data.Mdata.Pos    = Date[33] << 24 | Date[32] << 16 | Date[31] << 8 | Date[30];  // 反拼

        Date_left.motor_id = Date_left.motor_recv_data.head.motorID;                               // ID     正确
        Date_left.mode     = Date_left.motor_recv_data.Mdata.mode;                                 // mode   正确
        Date_left.Temp     = Date_left.motor_recv_data.Mdata.Temp;                                 // Temp   正确 (整数)
        Date_left.MError   = Date_left.motor_recv_data.Mdata.MError;                               // MError 正确
        Date_left.T        = (float) Date_left.motor_recv_data.Mdata.T / 256;                      // T      正确
        Date_left.Pos      = (float) (Date_left.motor_recv_data.Mdata.Pos / (16384.0f/2/PI));      // Pos    正确
        Date_left.W        = (float) Date_left.motor_recv_data.Mdata.W / 128;                      // W      正确 (小数)
        Date_left.Acc      = (float) Date_left.motor_recv_data.Mdata.Acc;                          // Acc    貌似正确 (需要VOFA打印测试看是否连续)

        if (Date_left.motor_id == 0x00)
        {
            MotorA1_recv_left_id00.motor_id = Date_left.motor_id;
            MotorA1_recv_left_id00.mode     = Date_left.mode; 
            MotorA1_recv_left_id00.Temp     = Date_left.Temp;
            MotorA1_recv_left_id00.MError   = Date_left.MError;
            MotorA1_recv_left_id00.T        = Date_left.T * 9.1f;            // 减速后的扭矩
            MotorA1_recv_left_id00.W        = Date_left.W / 9.1f;            // 减速后的角速度
            MotorA1_recv_left_id00.Pos      = Date_left.Pos * (180/PI/9.1f); // 减速后的角度     Date_left.Pos = 减速前的弧度
            MotorA1_recv_left_id00.Acc      = Date_left.Acc; 
        }

        if (Date_left.motor_id == 0x01)
        {
            MotorA1_recv_left_id01.motor_id = Date_left.motor_id;
            MotorA1_recv_left_id01.mode     = Date_left.mode; 
            MotorA1_recv_left_id01.Temp     = Date_left.Temp;
            MotorA1_recv_left_id01.MError   = Date_left.MError;
            MotorA1_recv_left_id01.T        = Date_left.T * 9.1f;
            MotorA1_recv_left_id01.W        = Date_left.W / 9.1f;   
            MotorA1_recv_left_id01.Pos      = Date_left.Pos*(180/PI/9.1f);
            MotorA1_recv_left_id01.Acc      = Date_left.Acc; 

        }

        if (Date_left.motor_id == 0x02)
        {
            MotorA1_recv_left_id02.motor_id = Date_left.motor_id;
            MotorA1_recv_left_id02.mode     = Date_left.mode; 
            MotorA1_recv_left_id02.Temp     = Date_left.Temp;
            MotorA1_recv_left_id02.MError   = Date_left.MError;
            MotorA1_recv_left_id02.T        = Date_left.T * 9.1f;
            MotorA1_recv_left_id02.W        = Date_left.W / 9.1f;  
            MotorA1_recv_left_id02.Pos      = Date_left.Pos*(180/PI/9.1f);
            MotorA1_recv_left_id02.Acc      = Date_left.Acc; 
        }
    }

    /*—————————————————————————————————————————右腿代码范围————————————————————————————————————————————————————————*/
    if (huart == &huart6)
    {
        uint8_t A1MotorA1_send_right[34]; // 发送数据
        uint8_t Date[78];        // 接收数据

        // 此处为右腿一号电机结构体//
        MotorA1_send_right.motor_send_data.head.start[0] = 0xFE;
        MotorA1_send_right.motor_send_data.head.start[1] = 0xEE;
        MotorA1_send_right.motor_send_data.head.motorID  = MotorA1_send_left.id;
        MotorA1_send_right.motor_send_data.head.reserved = 0x00;

        MotorA1_send_right.motor_send_data.Mdata.mode = MotorA1_send_right.mode; // mode = 10
        MotorA1_send_right.motor_send_data.Mdata.ModifyBit = 0xFF;
        MotorA1_send_right.motor_send_data.Mdata.ReadBit   = 0x00;
        MotorA1_send_right.motor_send_data.Mdata.reserved  = 0x00;
        MotorA1_send_right.motor_send_data.Mdata.Modify.F  = 0;
        MotorA1_send_right.motor_send_data.Mdata.T         = MotorA1_send_right.T * 256;
        MotorA1_send_right.motor_send_data.Mdata.W         = MotorA1_send_right.W * 128;
        MotorA1_send_right.motor_send_data.Mdata.Pos       = (int)((MotorA1_send_right.Pos / 6.2832f) * 16384.0f);
        MotorA1_send_right.motor_send_data.Mdata.K_P       = MotorA1_send_right.K_P * 2048;
        MotorA1_send_right.motor_send_data.Mdata.K_W       = MotorA1_send_right.K_W * 1024;

        MotorA1_send_right.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
        MotorA1_send_right.motor_send_data.Mdata.LowHzMotorCmdByte  = 0;
        MotorA1_send_right.motor_send_data.Mdata.Res[0] = MotorA1_send_right.Res;

        MotorA1_send_right.motor_send_data.CRCdata.u32 = crc32_core_Ver3((uint32_t *)(&MotorA1_send_right.motor_send_data), 7); // CRC校验

        memcpy(A1MotorA1_send_right, &MotorA1_send_right.motor_send_data, 34);

        // DMA 发送数据 + 接收数据
        HAL_UART_Transmit(&huart6, A1MotorA1_send_right, 34,0x03);
        HAL_Delay(10);
        HAL_UART_Receive_DMA(&huart6, Date, 78);

        Date_right.motor_recv_data.head.motorID = Date[2];  
        Date_right.motor_recv_data.Mdata.mode   = Date[4];  
        Date_right.motor_recv_data.Mdata.Temp   = Date[6];
        Date_right.motor_recv_data.Mdata.MError = Date[7]; 
        Date_right.motor_recv_data.Mdata.T      = Date[13] << 8  | Date[12]; 
        Date_right.motor_recv_data.Mdata.W      = Date[15] << 8  | Date[14]; 
        Date_right.motor_recv_data.Mdata.Acc    = Date[27] << 8  | Date[26]; 
        Date_right.motor_recv_data.Mdata.Pos    = Date[33] << 24 | Date[32] << 16 | Date[31] << 8 | Date[30];  

        Date_right.motor_id = Date_right.motor_recv_data.head.motorID;                           
        Date_right.mode     = Date_right.motor_recv_data.Mdata.mode;                               
        Date_right.Temp     = Date_right.motor_recv_data.Mdata.Temp;                                
        Date_right.MError   = Date_right.motor_recv_data.Mdata.MError;                               
        Date_right.T        = (float) Date_right.motor_recv_data.Mdata.T / 256;                     
        Date_right.Pos      = (float) (Date_right.motor_recv_data.Mdata.Pos / (16384.0f/2/PI));      
        Date_right.W        = (float) Date_right.motor_recv_data.Mdata.W / 128;                      
        Date_right.Acc      = (float) Date_right.motor_recv_data.Mdata.Acc;                          

        if (Date_right.motor_id == 0x00)
        {
            MotorA1_recv_right_id00.motor_id = Date_right.motor_id;
            MotorA1_recv_right_id00.mode     = Date_right.mode; 
            MotorA1_recv_right_id00.Temp     = Date_right.Temp;
            MotorA1_recv_right_id00.MError   = Date_right.MError;
            MotorA1_recv_right_id00.T        = Date_right.T * 9.1f;
            MotorA1_recv_right_id00.W        = Date_right.W / 9.1f;   
            MotorA1_recv_right_id00.Pos      = Date_right.Pos*(180/PI/9.1f);
            MotorA1_recv_right_id00.Acc      = Date_right.Acc; 
        }

        if (Date_right.motor_id == 0x01)
        {
            MotorA1_recv_right_id01.motor_id = Date_right.motor_id;
            MotorA1_recv_right_id01.mode     = Date_right.mode; 
            MotorA1_recv_right_id01.Temp     = Date_right.Temp;
            MotorA1_recv_right_id01.MError   = Date_right.MError;
            MotorA1_recv_right_id01.T        = Date_right.T * 9.1f;
            MotorA1_recv_right_id01.W        = Date_right.W / 9.1f;   
            MotorA1_recv_right_id01.Pos      = Date_right.Pos*(180/PI/9.1f);
            MotorA1_recv_right_id01.Acc      = Date_right.Acc; 
        }

        if (Date_right.motor_id == 0x02)
        {
            MotorA1_recv_right_id02.motor_id = Date_right.motor_id;
            MotorA1_recv_right_id02.mode     = Date_right.mode; 
            MotorA1_recv_right_id02.Temp     = Date_right.Temp;
            MotorA1_recv_right_id02.MError   = Date_right.MError;
            MotorA1_recv_right_id02.T        = Date_right.T * 9.1f;
            MotorA1_recv_right_id02.W        = Date_right.W / 9.1f;   
            MotorA1_recv_right_id02.Pos      = Date_right.Pos*(180/PI/9.1f);
            MotorA1_recv_right_id02.Acc      = Date_right.Acc; 
        }
    }
}

// 电机0位函数

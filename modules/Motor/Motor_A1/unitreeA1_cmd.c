#include "main.h"
#include "motor_msg.h"
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "unitreeA1_cmd.h"

#define PI 3.14159

motor_send_t cmd_left;        // 左腿一号电机数据体
motor_send_t cmd_right;       // 右腿一号电机数据体

motor_recv_t Date_left;       // 左腿电机接收数据体
motor_recv_t id00_left_date;  // 左腿00号电机接收数据体
motor_recv_t id01_left_date;  // 左腿01号电机接收数据体
motor_recv_t id02_left_date;  // 左腿02号电机接收数据体

motor_recv_t Date_right;      // 右腿电机接收数据体
motor_recv_t id00_right_date; // 右腿00号电机接收数据体
motor_recv_t id01_right_date; // 右腿01号电机接收数据体
motor_recv_t id02_right_date; // 右腿02号电机接收数据体

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
void modfiy_cmd(motor_send_t *send,uint8_t id, float Pos, float KP, float KW)
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
    send->W    = Omega;
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
    send->T    = torque;
    send->K_P  = 0.0;
    send->K_W  = 0.0;
}

// 电机发送接收函数
void unitreeA1_rxtx(UART_HandleTypeDef *huart)
{
    /*—————————————————————————————————————————左腿代码范围————————————————————————————————————————————————*/
    if (huart == &huart1)
    {
        uint8_t A1cmd_left[34]; // 发送数据
        uint8_t Date[78];       // 接收数据

        // 此处为左腿电机结构体//
        cmd_left.motor_send_data.head.start[0] = 0xFE;
        cmd_left.motor_send_data.head.start[1] = 0xEE;
        cmd_left.motor_send_data.head.motorID  = cmd_left.id;
        cmd_left.motor_send_data.head.reserved = 0x00;

        cmd_left.motor_send_data.Mdata.mode      = cmd_left.mode;  // mode = 10
        cmd_left.motor_send_data.Mdata.ModifyBit = 0xFF;
        cmd_left.motor_send_data.Mdata.ReadBit   = 0x00;
        cmd_left.motor_send_data.Mdata.reserved  = 0x00;
        cmd_left.motor_send_data.Mdata.Modify.F  = 0;
        cmd_left.motor_send_data.Mdata.T         = cmd_left.T * 256;
        cmd_left.motor_send_data.Mdata.W         = cmd_left.W * 128;
        cmd_left.motor_send_data.Mdata.Pos       = (int)((cmd_left.Pos / 6.2832f) * 16384.0f); // 单位 rad
        cmd_left.motor_send_data.Mdata.K_P       = cmd_left.K_P * 2048;
        cmd_left.motor_send_data.Mdata.K_W       = cmd_left.K_W * 1024;

        cmd_left.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
        cmd_left.motor_send_data.Mdata.LowHzMotorCmdByte  = 0;
        cmd_left.motor_send_data.Mdata.Res[0] = cmd_left.Res;

        cmd_left.motor_send_data.CRCdata.u32 = crc32_core_Ver3((uint32_t *)(&cmd_left.motor_send_data), 7); // CRC校验

        memcpy(A1cmd_left, &cmd_left.motor_send_data, 34);
        
        // HAL库 DMA 发送数据 + 接收数据
        HAL_UART_Transmit_DMA(&huart1, A1cmd_left, 34);
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
            id00_left_date.motor_id = Date_left.motor_id;
            id00_left_date.mode     = Date_left.mode; 
            id00_left_date.Temp     = Date_left.Temp;
            id00_left_date.MError   = Date_left.MError;
            id00_left_date.T        = Date_left.T;
            id00_left_date.W        = Date_left.W;   
            id00_left_date.Pos      = Date_left.Pos;
            id00_left_date.Acc      = Date_left.Acc; 
        }

        if (Date_left.motor_id == 0x01)
        {
            id01_left_date.motor_id = Date_left.motor_id;
            id01_left_date.mode     = Date_left.mode; 
            id01_left_date.Temp     = Date_left.Temp;
            id01_left_date.MError   = Date_left.MError;
            id01_left_date.T        = Date_left.T;
            id01_left_date.W        = Date_left.W;   
            id01_left_date.Pos      = Date_left.Pos;
            id01_left_date.Acc      = Date_left.Acc; 

        }

        if (Date_left.motor_id == 0x02)
        {
            id02_left_date.motor_id = Date_left.motor_id;
            id02_left_date.mode     = Date_left.mode; 
            id02_left_date.Temp     = Date_left.Temp;
            id02_left_date.MError   = Date_left.MError;
            id02_left_date.T        = Date_left.T;
            id02_left_date.W        = Date_left.W;  
            id02_left_date.Pos      = Date_left.Pos;
            id02_left_date.Acc      = Date_left.Acc; 
        }
    }

    /*—————————————————————————————————————————右腿代码范围————————————————————————————————————————————————————————*/
    if (huart == &huart6)
    {
        uint8_t A1cmd_right[34]; // 发送数据
        uint8_t Date[78];        // 接收数据

        // 此处为右腿一号电机结构体//
        cmd_right.motor_send_data.head.start[0] = 0xFE;
        cmd_right.motor_send_data.head.start[1] = 0xEE;
        cmd_right.motor_send_data.head.motorID  = cmd_left.id;
        cmd_right.motor_send_data.head.reserved = 0x00;

        cmd_right.motor_send_data.Mdata.mode = cmd_right.mode; // mode = 10
        cmd_right.motor_send_data.Mdata.ModifyBit = 0xFF;
        cmd_right.motor_send_data.Mdata.ReadBit   = 0x00;
        cmd_right.motor_send_data.Mdata.reserved  = 0x00;
        cmd_right.motor_send_data.Mdata.Modify.F  = 0;
        cmd_right.motor_send_data.Mdata.T         = cmd_right.T * 256;
        cmd_right.motor_send_data.Mdata.W         = cmd_right.W * 128;
        cmd_right.motor_send_data.Mdata.Pos       = (int)((cmd_right.Pos / 6.2832f) * 16384.0f);
        cmd_right.motor_send_data.Mdata.K_P       = cmd_right.K_P * 2048;
        cmd_right.motor_send_data.Mdata.K_W       = cmd_right.K_W * 1024;

        cmd_right.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
        cmd_right.motor_send_data.Mdata.LowHzMotorCmdByte  = 0;
        cmd_right.motor_send_data.Mdata.Res[0] = cmd_right.Res;

        cmd_right.motor_send_data.CRCdata.u32 = crc32_core_Ver3((uint32_t *)(&cmd_right.motor_send_data), 7); // CRC校验

        memcpy(A1cmd_right, &cmd_right.motor_send_data, 34);

        // DMA 发送数据 + 接收数据
        HAL_UART_Transmit(&huart6, A1cmd_right, 34,0x03);
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
            id00_right_date.motor_id = Date_right.motor_id;
            id00_right_date.mode     = Date_right.mode; 
            id00_right_date.Temp     = Date_right.Temp;
            id00_right_date.MError   = Date_right.MError;
            id00_right_date.T        = Date_right.T;
            id00_right_date.W        = Date_right.W;   
            id00_right_date.Pos      = Date_right.Pos;
            id00_right_date.Acc      = Date_right.Acc; 
        }

        if (Date_right.motor_id == 0x01)
        {
            id01_right_date.motor_id = Date_right.motor_id;
            id01_right_date.mode     = Date_right.mode; 
            id01_right_date.Temp     = Date_right.Temp;
            id01_right_date.MError   = Date_right.MError;
            id01_right_date.T        = Date_right.T;
            id01_right_date.W        = Date_right.W;   
            id01_right_date.Pos      = Date_right.Pos;
            id01_right_date.Acc      = Date_right.Acc; 
        }

        if (Date_right.motor_id == 0x02)
        {
            id02_right_date.motor_id = Date_right.motor_id;
            id02_right_date.mode     = Date_right.mode; 
            id02_right_date.Temp     = Date_right.Temp;
            id02_right_date.MError   = Date_right.MError;
            id02_right_date.T        = Date_right.T;
            id02_right_date.W        = Date_right.W;   
            id02_right_date.Pos      = Date_right.Pos;
            id02_right_date.Acc      = Date_right.Acc; 
        }
    }
}

// 电机0位函数

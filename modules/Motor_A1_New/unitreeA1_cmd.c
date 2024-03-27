#include "main.h"
#include "motor_msg.h"
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "unitreeA1_cmd.h"

motor_send_t cmd_left;  // 左腿一号电机数据体
motor_send_t cmd_right; // 右腿一号电机数据体

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

// 电机位置位置修改
void modfiy_cmd(motor_send_t *send,uint8_t id, float Pos, float KP, float KW)
{

    send->hex_len = 34;

    send->mode = 10;
	send->id   = id;

    send->Pos  = 6.2832*9.1*2*Pos;  // 6.2832 = 2 PI
    send->W    = 0;
    send->T    = 0.0;
    send->K_P  = KP;
    send->K_W  = KW;
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
        cmd_left.motor_send_data.Mdata.Pos       = (int)((cmd_left.Pos / 6.2832f) * 16384.0f);
        cmd_left.motor_send_data.Mdata.K_P       = cmd_left.K_P * 2048;
        cmd_left.motor_send_data.Mdata.K_W       = cmd_left.K_W * 1024;

        cmd_left.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
        cmd_left.motor_send_data.Mdata.LowHzMotorCmdByte  = 0;
        cmd_left.motor_send_data.Mdata.Res[0] = cmd_left.Res;

        cmd_left.motor_send_data.CRCdata.u32 = crc32_core_Ver3((uint32_t *)(&cmd_left.motor_send_data), 7); // CRC校验

        memcpy(A1cmd_left, &cmd_left.motor_send_data, 34);
        
        // DMA 发送数据 + 接收数据
        HAL_UART_Transmit_DMA(&huart1, A1cmd_left, 34);
        HAL_Delay(10);
        HAL_UART_Receive_DMA(&huart1, Date, 78);

        // 接受数据处理
        Date_left.motor_recv_data.head.motorID = Date[2];
        Date_left.motor_recv_data.Mdata.MError = Date[7];
        Date_left.motor_recv_data.Mdata.T      = Date[12] << 8  | Date[13];
        Date_left.motor_recv_data.Mdata.Pos2   = Date[30] << 24 | Date[31] << 16 | Date[32] << 8 | Date[33];

        Date_left.motor_id = Date_left.motor_recv_data.head.motorID;
        Date_left.MError   = Date_left.motor_recv_data.Mdata.MError;
        Date_left.T        = Date_left.motor_recv_data.Mdata.T / 256;
        Date_left.Pos      = (int)((Date_left.motor_recv_data.Mdata.Pos2 / 16384.0f) * 6.2832f);

        if (Date_left.motor_id == 0x00)
        {
            id00_left_date.motor_id = Date_left.motor_id;
            id00_left_date.MError   = Date_left.MError;
            id00_left_date.T        = Date_left.T;
            id00_left_date.Pos      = Date_left.Pos;
        }

        if (Date_left.motor_id == 0x01)
        {
            id01_left_date.motor_id = Date_left.motor_id;
            id01_left_date.MError   = Date_left.MError;
            id01_left_date.T        = Date_left.T;
            id01_left_date.Pos      = Date_left.Pos;
        }

        if (Date_left.motor_id == 0x02)
        {
            id02_left_date.motor_id = Date_left.motor_id;
            id02_left_date.MError   = Date_left.MError;
            id02_left_date.T        = Date_left.T;
            id02_left_date.Pos      = Date_left.Pos;
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
        Date_right.motor_recv_data.Mdata.MError = Date[7];
        Date_right.motor_recv_data.Mdata.T      = Date[12] << 8  | Date[13];
        Date_right.motor_recv_data.Mdata.Pos2   = Date[30] << 24 | Date[31] << 16 | Date[32] << 8 | Date[33];

        Date_right.motor_id = Date_right.motor_recv_data.head.motorID;
        Date_right.MError   = Date_right.motor_recv_data.Mdata.MError;
        Date_right.T        = Date_right.motor_recv_data.Mdata.T / 256;
        Date_right.Pos      = (int)((Date_right.motor_recv_data.Mdata.Pos2 / 16384.0f) * 6.2832f);

        if (Date_right.motor_id == 0x00)
        {
            id00_right_date.motor_id = Date_right.motor_id;
            id00_right_date.MError = Date_right.MError;
            id00_right_date.T = Date_right.T;
            id00_right_date.Pos = Date_right.Pos;
        }

        if (Date_right.motor_id == 0x01)
        {
            id01_right_date.motor_id = Date_right.motor_id;
            id01_right_date.MError = Date_right.MError;
            id01_right_date.T = Date_right.T;
            id01_right_date.Pos = Date_right.Pos;
        }

        if (Date_right.motor_id == 0x02)
        {
            id02_right_date.motor_id = Date_right.motor_id;
            id02_right_date.MError = Date_right.MError;
            id02_right_date.T = Date_right.T;
            id02_right_date.Pos = Date_right.Pos;
        }
    }
}

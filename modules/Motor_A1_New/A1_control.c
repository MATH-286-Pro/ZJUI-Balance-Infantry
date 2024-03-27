#include "main.h"
#include "usart.h" 
#include "can.h"
#include "A1_control.h"
UintreeA1_DateTypDef UintreeA1;

// CRC校验位的函数
uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
	uint32_t xbit = 0;
	uint32_t data = 0;
	uint32_t CRC32 = 0xFFFFFFFF;
	const uint32_t dwPolynomial = 0x04c11db7;
	for (uint32_t i = 0; i < len; i++)
	{
		xbit = 1 << 31;
		data = ptr[i];
		for (uint32_t bits = 0; bits < 32; bits++)
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

uint32_t UintreeA1_Cmd[16];
uint32_t UintreeA1_Cmd02 ;
uint8_t UintreeA1_Cmd01[34] = {0};

/*—————————————————————————————————————————数据发送————————————————————————————————————————————————————————*/
// 宇树 A1 电机混合控制代码
// UnitreeA1_Cmd
// UnitreeA1_Cmd01
// UnitreeA1_Cmd02
void UintreeA1_control(uint8_t ID , uint8_t Mode, double T , double W , double Pos1 , double Kp, double Kd)
{	
	// 数据处理
	UintreeA1.T_256    = T;
	uint16_t W_128     = W * 128;
	uint32_t Pos_16384 = Pos1 * 16384 / 6.2831;
	uint16_t Kp_2048   = Kp * 2048;
	uint16_t Kd_1024   = Kd * 1024;

	
	// 数据包头 //
	UintreeA1_Cmd01[0] = 0xFE;
	UintreeA1_Cmd01[1] = 0xEE;
	UintreeA1_Cmd01[2] = ID;
	UintreeA1_Cmd01[3] = 0x00;

	UintreeA1_Cmd[0] = (uint32_t)(UintreeA1_Cmd01[0] << 24 |  (uint32_t)UintreeA1_Cmd01[1] << 16 |(uint32_t)UintreeA1_Cmd01[2] << 8 |(uint32_t)UintreeA1_Cmd01[3] << 0 );

	// 数据体 //
	UintreeA1_Cmd01[4] = Mode ;     //mode
	UintreeA1_Cmd01[5] = 0x00 ;
	UintreeA1_Cmd01[6] = 0x00 ; 	
	UintreeA1_Cmd01[7] = 0x00 ; 
	UintreeA1_Cmd01[8] = 0x00 ; 
	UintreeA1_Cmd01[9] = 0x00 ; 
	UintreeA1_Cmd01[10] = 0x00 ; 
	UintreeA1_Cmd01[11] = 0x00 ;

	UintreeA1_Cmd[1] = (uint32_t)(UintreeA1_Cmd01[4] << 24 |(uint32_t)UintreeA1_Cmd01[5] << 16 | (uint32_t)UintreeA1_Cmd01[6] << 8 | (uint32_t)UintreeA1_Cmd01[7] << 0) ;
	UintreeA1_Cmd[2] = (uint32_t)(UintreeA1_Cmd01[8] << 24 |(uint32_t)UintreeA1_Cmd01[9] << 16 | (uint32_t)UintreeA1_Cmd01[10] << 8 | (uint32_t)UintreeA1_Cmd01[11] << 0) ;
	//参数//

	//电机前馈力矩 τff，×256 倍描述//
	UintreeA1_Cmd01[12] = (uint8_t)(UintreeA1.T_256 >> 8) ; 
	UintreeA1_Cmd01[13] = (uint8_t)(UintreeA1.T_256) ; 
	//电机速度命令 ωdes，×128 倍描述//
	UintreeA1_Cmd01[14] = (uint8_t)(W_128 >> 8); 
	UintreeA1_Cmd01[15] = (uint8_t)W_128 ;
	//电机位置命令 pdes，×16384/2π 倍描述//
	UintreeA1_Cmd01[16] = (uint8_t)(Pos_16384 >> 24);
	UintreeA1_Cmd01[17] = (uint8_t)(Pos_16384 >> 16);
	UintreeA1_Cmd01[18] = (uint8_t)(Pos_16384 >> 8);
	UintreeA1_Cmd01[19] = (uint8_t)Pos_16384;

	UintreeA1_Cmd[3] = (uint32_t)(UintreeA1_Cmd01[12] << 24 |(uint32_t)UintreeA1_Cmd01[13] << 16 | (uint32_t)UintreeA1_Cmd01[14] << 8 | (uint32_t)UintreeA1_Cmd01[15] << 0) ;
	UintreeA1_Cmd[4] = (uint32_t)(UintreeA1_Cmd01[16] << 24 |(uint32_t)UintreeA1_Cmd01[17] << 16 | (uint32_t)UintreeA1_Cmd01[18] << 8 | (uint32_t)UintreeA1_Cmd01[19] << 0) ;

	//电机位置刚度 kp，×2048 倍描述
	UintreeA1_Cmd01[20] = (uint8_t)(Kp_2048 >> 8) ;
	UintreeA1_Cmd01[21] = (uint8_t)Kp_2048  ;
	//电机速度刚度 kd，×1024 倍描述
	UintreeA1_Cmd01[22] = (uint8_t)(Kd_1024 >> 8) ;
	UintreeA1_Cmd01[23] = (uint8_t)Kd_1024 ;
	
	UintreeA1_Cmd[5] = (uint32_t)(UintreeA1_Cmd01[20] << 24 |(uint32_t)UintreeA1_Cmd01[21] << 16 | (uint32_t)UintreeA1_Cmd01[22] << 8 | (uint32_t)UintreeA1_Cmd01[23] << 0) ;

	//可忽略部分//
	UintreeA1_Cmd01[24] = 0x00 ;
	UintreeA1_Cmd01[25] = 0x00 ;
	UintreeA1_Cmd01[26] = 0x00 ;
	UintreeA1_Cmd01[27] = 0x00 ;

	UintreeA1_Cmd[6] = 0;
	UintreeA1_Cmd[7] = 0;

	UintreeA1_Cmd01[28] = 0x00 ;
	UintreeA1_Cmd01[29] = 0x00 ;
	
	//CRC检测计算
	
	UintreeA1_Cmd02 = crc32_core(UintreeA1_Cmd,7);
	
	UintreeA1_Cmd01[30] = UintreeA1_Cmd02 >> 24;
	UintreeA1_Cmd01[31] = UintreeA1_Cmd02 >> 16;
	UintreeA1_Cmd01[32] = UintreeA1_Cmd02 >> 8;
	UintreeA1_Cmd01[33] = UintreeA1_Cmd02 >> 0;
	
	// 发送数据
	HAL_UART_Transmit(&huart1,UintreeA1_Cmd01,34,0xff);
	// HAL_UART_Transmit(&huart6,UintreeA1_Cmd01,34,0xff);
}


/*—————————————————————————————————————————数据接收————————————————————————————————————————————————————————*/
uint8_t UintreeA1_Date[78] = {0};
void UintreeA1_Receive (void)
{
	HAL_UART_Receive(&huart1,UintreeA1_Date,sizeof(UintreeA1_Date),100);

	uint8_t Temp   = 0;
	uint8_t MError = 0;

	uint16_t T_256 = 0;
	uint16_t W_128 = 0;
	uint16_t Acc   = 0;
	uint32_t Pos   = 0;

	Temp   = UintreeA1_Date[6];
	MError = UintreeA1_Date[7];
	T_256  = UintreeA1_Date[12] << 8 | UintreeA1_Date[13] ;
	W_128  = UintreeA1_Date[14] << 8 | UintreeA1_Date[15] ;
	Acc    = UintreeA1_Date[26] << 8 | UintreeA1_Date[27] ;
	Pos    = UintreeA1_Date[30] << 24 | UintreeA1_Date[31] << 16 | UintreeA1_Date[32] << 8 | UintreeA1_Date[33] ;

}

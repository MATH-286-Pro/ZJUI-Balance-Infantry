#include "motor.h"
#include "CRC.h" 
#include "usart.h"

//uint64_t tmp_data_1[34];
//COMHead comhead;
//MasterComdV3 comdv3;
//ServoComdDataV3 CRC32;

#define PI 3.1415926535f
int flag_IDD;
//void Chassis_UART_TX(void)
//{
//    tmp_data_1[0]= comhead.start[0]=0xFE;                                           
//    tmp_data_1[1]= comhead.start[1]=0xEE; 
//	  tmp_data_1[2]= comhead.motorID =0xBB;
//    tmp_data_1[4]= comdv3.mode = 5;
//	  comdv3.T = 2560;
//	  tmp_data_1[12]=(uint8_t)comdv3.T>>8;
//	  tmp_data_1[13]= (uint8_t)comdv3.T;
//	  comdv3.W = 1280;
//	  tmp_data_1[14]= comdv3.W >> 8;
//	  tmp_data_1[15]= comdv3.W;
//	  crc32_core((uint32_t*)tmp_data_1,7);
//	  tmp_data_1[30]=(comdv3.COMData32.u32>> 32);
//		tmp_data_1[31]=(comdv3.COMData32.u32>> 16);
//		tmp_data_1[32]=(comdv3.COMData32.u32>> 8);
//		tmp_data_1[33]=(comdv3.COMData32.u32);
//	  HAL_GPIO_WritePin(GPIOC, RS485_DIR1_Pin, GPIO_PIN_SET);
//    HAL_UART_Transmit_IT(&huart1,tmp_data_1,34);
//}
union Motor_Tx{
		uint8_t data[24];
		Send_Data Tx_Message;
}Motor_Tx_u;

union CRCC{
		uint8_t data[4];
		uint32_t crc;
}CRC_u;

uint8_t Data_Box[3][34];

uint32_t crc32_core(uint32_t *ptr, uint32_t len) {
	uint32_t xbit = 0;
	uint32_t data = 0;
	uint32_t CRC32 = 0xFFFFFFFF;
	const uint32_t dwPolynomial = 0x04c11db7;
	for (uint32_t i = 0; i < len; i++) {
		xbit = 1 << 31;
		data = ptr[i];
		for (uint32_t bits = 0; bits < 32; bits++) {
			if (CRC32 & 0x80000000) {
				CRC32 <<= 1;
				CRC32 ^= dwPolynomial;
			} else
				CRC32 <<= 1;
			if (data & xbit)
				CRC32 ^= dwPolynomial;
			xbit >>= 1;
		}
	}
	return CRC32;
}

void Control_Message_Send(int ID)
{
	uint32_t crc = crc32_core((uint32_t *) Data_Box[ID], 7);
	CRC_u.crc = crc;
	for (int i=0;i<24;i++) {Data_Box[ID][i] = Motor_Tx_u.data[i];}
	int cnt = 0;
	for (int i=30;i<34;i++){Data_Box[ID][i] = CRC_u.data[cnt++];}
	
	flag_IDD = ID;
	HAL_GPIO_WritePin(GPIOC, RS485_DIR1_Pin, GPIO_PIN_SET);
   HAL_UART_Transmit_IT(&huart1,Data_Box[ID],34);
	//HAL_GPIO_WritePin(GPIOC, RS485_DIR1_Pin, GPIO_PIN_RESET);
}

void Mode_Control(int ID,int Mode)
{
	Motor_Tx_u.Tx_Message.mode = Mode;
	Motor_Tx_u.Tx_Message.Motor_ID = ID;
	Motor_Tx_u.Tx_Message.start[0] = 0xFE;
	Motor_Tx_u.Tx_Message.start[1] = 0xEE;
}

//ç”µæœºæ··åˆæŽ§åˆ¶
void A1_Motor_Multiple_Control(int ID,int mode,float Torque,float W,float Position)
{
	if (mode == 1 || mode == 5)
	{
		Mode_Control(ID,mode);
		Control_Message_Send(ID);
		return;
	}
	Mode_Control(ID,10);
	// ¦Ó = ¦Óf f + kp ¡¤ (pdes ? p) + kd ¡¤ (¦Ødes ? ¦Ø)
	float kp = 0.2f;
	float kd = 0.5f;

	Motor_Tx_u.Tx_Message.T = (uint16_t)(Torque * 256.0f);
	Motor_Tx_u.Tx_Message.W = (uint16_t)(W * 128.0f * 9.1f);
	Motor_Tx_u.Tx_Message.Pos = (uint32_t)(Position * (16384.0f / (2.0f * PI)) * 9.1f);
//	Motor_Tx_u.Tx_Message.kp = (uint16_t)(0.2f*2048.0f);
//	Motor_Tx_u.Tx_Message.kw = (uint16_t)(3.0f*1024.0f);
	Control_Message_Send(ID);
}

//ç”µæœºé€Ÿåº¦æŽ§åˆ¶
void A1_Motor_Speed_Control(int ID,float W)
{
	Mode_Control(ID,10);
	Motor_Tx_u.Tx_Message.T = 0;
	Motor_Tx_u.Tx_Message.W = 	(uint16_t)(W * 128.0f * 9.1f);
	Motor_Tx_u.Tx_Message.Pos = 0;
	Motor_Tx_u.Tx_Message.kw = (uint16_t)(3.0f * 1024.0f);
	Control_Message_Send(ID);
}

//ç”µæœºä½ç½®æŽ§åˆ¶
void A1_Motor_Position_Control(int ID,float Position)
{
	Mode_Control(ID,10);
	Motor_Tx_u.Tx_Message.T = 0;
	Motor_Tx_u.Tx_Message.W = 0;
	Motor_Tx_u.Tx_Message.Pos = (uint32_t)(Position * (16384.0f / (2.0f * PI)) * 9.1f);
	Motor_Tx_u.Tx_Message.kp = (uint16_t)(0.2f * 2048.0f);
	Motor_Tx_u.Tx_Message.kw = (uint16_t)(3.0f * 1024.0f);
	Control_Message_Send(ID);
}

#define SBUS_RX_BUF_NUM 99u

uint8_t A1_Motor_Rx_Data[2][SBUS_RX_BUF_NUM];

union Motor_Rx{
		uint8_t data[12];
		Rx_Data Received_data;
}Motor_Rx_u;

Motor_State A1_State;

void Received_Data_Dealer(const uint8_t *sbus_buf)
{
	int ID = sbus_buf[2];
	Motor_Rx_u.data[0] = sbus_buf[4];
	Motor_Rx_u.data[1] = sbus_buf[6];
	Motor_Rx_u.data[2] = sbus_buf[12];
	Motor_Rx_u.data[3] = sbus_buf[13];
	Motor_Rx_u.data[4] = sbus_buf[14];
	Motor_Rx_u.data[5] = sbus_buf[15];
	Motor_Rx_u.data[6] = sbus_buf[26];
	Motor_Rx_u.data[7] = sbus_buf[27];
	Motor_Rx_u.data[8] = sbus_buf[30];
	Motor_Rx_u.data[9] = sbus_buf[31];
	Motor_Rx_u.data[10] = sbus_buf[32];
	Motor_Rx_u.data[11] = sbus_buf[33];
	A1_State.Mode = Motor_Rx_u.Received_data.mode;
	A1_State.Temp = Motor_Rx_u.Received_data.Temp;
	A1_State.Torque = ((float)Motor_Rx_u.Received_data.T)/256.0f;
	A1_State.Omega = ((float)Motor_Rx_u.Received_data.W)/128.0f;
	A1_State.Acc = Motor_Rx_u.Received_data.Acc;
	A1_State.Position = ((float)Motor_Rx_u.Received_data.Pos*2.0f*PI)/(16384.0f);
}


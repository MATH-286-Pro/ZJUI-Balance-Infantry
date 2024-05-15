#ifndef _RC_H_
#define _RC_H_

/* ------------------------------ Include ------------------------------ */
#include "stdint.h"
/* ------------------------------ Macro Definition ------------------------------ */
#define SW_POWER_OFF 0x00
#define SW_UP        0x01
#define SW_MID       0x03
#define SW_DOWN      0x02
#define DBUS_OFFLINE_TICK   200  // 遥控器超时时间，单位为ms
#pragma pack()  // 按照1字节对齐
/* ------------------------------ Type Definition ------------------------------ */
typedef struct {
	float LY; //[-1,1]
	float LX; //[-1,1]
	float RY; //[-1,1]
	float RX; //[-1,1]
	// ch value: -660 ~ 660
	int16_t ch1;	
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	
	// switch value: 1 3 2
	uint8_t sw1;	
	uint8_t sw2;
	
	struct {
		int16_t x;
		int16_t y;
		int16_t z; //no use
	
		// press:1 release:0
		uint8_t l;
		uint8_t r;
	} mouse;
	
	union {  // 共用体，key_code和bit共享一段内存
		uint16_t key_code;
/**********************************************************************************
 * keyboard: 15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 *            key_V    key_C    key_X	 key_Z    key_G    key_F   key_R   key_E   key_Q  key_CTRL key_SHIFT   key_D   key_A   key_S   key_W
 ************************************************************************************/
		struct {
			uint16_t key_W: 1;
			uint16_t key_S: 1;
			uint16_t key_A: 1;
			uint16_t key_D: 1;
			uint16_t key_SHIFT: 1;
			uint16_t key_CTRL: 1;
			uint16_t key_Q: 1;
			uint16_t key_E: 1;
			uint16_t key_R: 1;
			uint16_t key_F: 1;
			uint16_t key_G: 1;
			uint16_t key_Z: 1;
			uint16_t key_X: 1;
			uint16_t key_C: 1;
			uint16_t key_V: 1;
			uint16_t key_B: 1;
		} bit;
	} kb;
    
    int16_t wheel;  //左上角拨轮 //364~1024~1684
} RC_Type;

/* ------------------------------ Extern Global Variable ------------------------------ */
extern RC_Type rc;
extern uint32_t rc_tick;

/* ------------------------------ Function Declaration (used in other .c files) ------------------------------ */
void Dbus_Init(void); //放在main
void Dbus_UART_IRQHandler(void); //放在中断处理函数中

#endif

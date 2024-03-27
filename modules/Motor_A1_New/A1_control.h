#ifndef __A1_CONTROL_H
#define __A1_CONTROL_H

#include "main.h"

extern	uint8_t Temp ;
extern	uint8_t MError;
extern	uint16_t T_256 ;
extern	uint16_t W_128 ;
extern	uint16_t Acc ;
extern	uint32_t Pos ;

void UintreeA1_control(uint8_t ID , uint8_t Mode, double T , double W , double Pos , double Kp, double Kd);
void UintreeA1_Receive (void);
uint32_t crc32_core(uint32_t* ptr, uint32_t len);


typedef struct
{
	uint8_t Temp ;

	uint8_t MError;

	uint16_t T_256 ;

	uint16_t W_128 ;

	uint16_t Acc ;

	uint32_t Pos ;

}UintreeA1_DateTypDef;
extern UintreeA1_DateTypDef UintreeA1;

#endif

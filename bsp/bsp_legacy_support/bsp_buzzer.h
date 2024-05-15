#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include <stdint.h>

void BuzzerInit();
extern void BuzzerOn();
extern void BuzzerOff(void);
extern void Buzzer_beep(void);

#endif

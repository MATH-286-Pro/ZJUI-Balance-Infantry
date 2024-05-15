#ifndef DEBUG_TOOL
#define DEBUG_TOOL

extern void usart1_printf(const char *fmt,...);

extern int mapRange(int value, int fromMin, int fromMax, int toMin, int toMax);

extern void trackMinMax(int newValue);

void USB_printf(const char *format, ...);

#endif
#include <stdio.h>
#include <stdarg.h>
#include "main.h"
#include "Debug_tool.h"
#include "usart.h"
#include "usbd_cdc_if.h"

//定义 Uart1 串口打印函数(调试工具)
void usart1_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    //return length of string 
    len = vsprintf((char *)tx_buf, fmt, ap); //发送信息长度

    va_end(ap);

    HAL_UART_Transmit_DMA(&huart1,tx_buf, len);

}

//定义mapRange 函数(调试工具)
int mapRange(int value, int fromMin, int fromMax, int toMin, int toMax) 
{
    return (value - fromMin) * (toMax - toMin) / (fromMax - fromMin) + toMin;
}

//定义 trackMinMax 函数(调试工具)
//跟踪数值最大最小值
void trackMinMax(int newValue) 
{
    static int minValue=0;
    static int maxValue=0;
    if (newValue < minValue) {
        minValue = newValue;
    }
    if (newValue > maxValue) {
        maxValue = newValue;
    }
    usart1_printf("Current x: %d, Min x: %d, Max x: %d\n", newValue, minValue, maxValue);

}

// 使用USB发送数据
void USB_printf(const char *format, ...) {
    char buffer[256]; // 定义一个足够大的缓冲区来存储生成的字符串
    va_list args;     // 定义一个 va_list 类型变量，用来处理可变参数

    va_start(args, format); // 初始化 args 变量，使其指向可变参数表中的第一个参数
    vsnprintf(buffer, sizeof(buffer), format, args); // 使用 vsnprintf 将格式化的数据写入缓冲区
    va_end(args); // 清理工作：结束可变参数的获取

    CDC_Transmit_FS(buffer, strlen(buffer)); // 使用 CDC_Transmit_FS 发送缓冲区内容
}
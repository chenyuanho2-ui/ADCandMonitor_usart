////用于printf的重定向

#include <stdio.h>
#include "main.h"   // 引入这个是为了能找到 HAL 库的定义
#include "usart.h"  // 引入这个是为了能找到 huart1 的定义

// 告诉编译器，huart1 这个变量是在别的地方定义的（通常在 usart.c 或 main.c 里）
// 如果你的 usart.h 里已经有了 extern UART_HandleTypeDef huart1; 这句话可以省略
// 但为了保险起见，加上也没错
extern UART_HandleTypeDef huart1;

// 重定向 fputc 函数
// 这是 printf 的底层实现，printf 每打印一个字符，都会调用一次 fputc
int fputc(int ch, FILE *f) {
  // 通过串口1发送字符
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

// 可选：如果你将来想用 scanf 读取串口数据，可以把这个也加上
int fgetc(FILE *f) {
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xFFFF);
  return ch;
}

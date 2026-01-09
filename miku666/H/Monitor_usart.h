/*
 * Monitor_usart.h
 */
#ifndef MONITOR_USART_H
#define MONITOR_USART_H

#include "main.h"

// 功能函数声明
void Monitor_Init(void);   // 初始化
void Monitor_Task(void);   // 在主循环中调用，用于ADC定时采样

#endif /* MONITOR_USART_H */

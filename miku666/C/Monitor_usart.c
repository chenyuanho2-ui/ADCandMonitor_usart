/*
 * Monitor_usart.c
 * 最终修正版：
 * 1. 协议偏移修正：FC [Skip] [Skip] [Skip] [LSB] [MSB]
 * (读取第5、6字节作为温度)
 * 2. 温度计算：Hex / 10.0 = Temp
 * 3. 按键逻辑：PA3 按一下开始，再按一下停止
 */

#include "Monitor_usart.h"
#include "usart.h"
#include "adc.h"
#include "stdio.h"
#include "string.h"

// 引用外部句柄
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

// ================= 配置参数 =================
#define PRINT_INTERVAL      250  // 打印间隔：250ms
#define ADC_SAMPLE_INTERVAL 50   // ADC采样间隔：50ms
#define ADC_WINDOW_SIZE     5    // 滤波窗口大小

// ================= 全局变量 =================

// --- 串口协议相关 ---
#define RX_BUFFER_SIZE 1
static uint8_t rx_byte;
static float latest_temp_val = 0.0f;   // 存储解析出的实际温度值(浮点)
static uint8_t temp_received_flag = 0; // 标记是否收到过有效温度

// 协议状态机
typedef enum {
    STATE_WAIT_FC,      // 等待包头 FC (Byte 1)
    STATE_SKIP_BYTE_2,  // 跳过 Byte 2
    STATE_SKIP_BYTE_3,  // 跳过 Byte 3
    STATE_SKIP_BYTE_4,  // 跳过 Byte 4
    STATE_READ_T_LSB,   // 读取 Byte 5 (温度低位, 如 4C)
    STATE_READ_T_MSB    // 读取 Byte 6 (温度高位, 如 01)
} ParseState_t;

static ParseState_t current_state = STATE_WAIT_FC;
static uint8_t temp_lsb = 0;

// --- ADC 滤波与按键相关 ---
static uint16_t adc_window[ADC_WINDOW_SIZE] = {0};
static uint8_t adc_win_idx = 0;
static uint32_t last_adc_tick = 0;

// 按键控制
static uint8_t is_printing = 0;         // 打印开关 (0:停, 1:打印)
static uint8_t last_btn_state = GPIO_PIN_SET; // 上次按键状态

// 定时打印
static uint32_t next_print_tick = 0;

// ================= 内部工具函数 =================

static uint16_t Get_Median_ADC(void) {
    uint16_t temp[ADC_WINDOW_SIZE];
    for(int i=0; i<ADC_WINDOW_SIZE; i++) temp[i] = adc_window[i];
    
    // 冒泡排序
    for (int i = 0; i < ADC_WINDOW_SIZE - 1; i++) {
        for (int j = 0; j < ADC_WINDOW_SIZE - i - 1; j++) {
            if (temp[j] > temp[j+1]) {
                uint16_t t = temp[j]; temp[j] = temp[j+1]; temp[j+1] = t;
            }
        }
    }
    return temp[ADC_WINDOW_SIZE / 2];
}

// ================= 核心接口 =================

void Monitor_Init(void) {
    // 1. 启动硬件
    HAL_ADC_Start(&hadc1);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    
    // 2. 初始化变量
    uint32_t now = HAL_GetTick();
    last_adc_tick = now;
    next_print_tick = now + PRINT_INTERVAL;
    
    // 预填充 ADC 窗口
    uint16_t initial_val = HAL_ADC_GetValue(&hadc1);
    for(int i=0; i<ADC_WINDOW_SIZE; i++) adc_window[i] = initial_val;

    // 上电提示
    char *msg = "\r\n[Offset Corrected] Reading 5th & 6th bytes.\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

// 主循环任务
void Monitor_Task(void) {
    uint32_t now = HAL_GetTick();

    // 1. ADC 采样
    if (now - last_adc_tick >= ADC_SAMPLE_INTERVAL) {
        last_adc_tick = now;
        adc_window[adc_win_idx++] = (uint16_t)HAL_ADC_GetValue(&hadc1);
        if (adc_win_idx >= ADC_WINDOW_SIZE) adc_win_idx = 0;
    }

    // 2. 按键检测 (自锁模式)
    uint8_t curr_btn_state = HAL_GPIO_ReadPin(BOTTON1_GPIO_Port, BOTTON1_Pin);
    if (curr_btn_state == GPIO_PIN_RESET && last_btn_state == GPIO_PIN_SET) {
        HAL_Delay(100); // 消抖
        if (HAL_GPIO_ReadPin(BOTTON1_GPIO_Port, BOTTON1_Pin) == GPIO_PIN_RESET) {
            is_printing = !is_printing;
            char *state_msg = is_printing ? "-> START\r\n" : "-> STOP\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)state_msg, strlen(state_msg), 100);
        }
    }
    last_btn_state = curr_btn_state;

    // 3. 打印逻辑
    if (now >= next_print_tick) {
        if (is_printing) {
            uint16_t median_adc = Get_Median_ADC();
            char msg[64];
            
            if (temp_received_flag) {
                // 打印格式：[时间戳] 温度值, ADC值
                sprintf(msg, "[%u ms] T:%.1f C, ADC:%u\r\n", 
                        next_print_tick, latest_temp_val, median_adc);
            } else {
                sprintf(msg, "[%u ms] T:Wait.., ADC:%u\r\n", 
                        next_print_tick, median_adc);
            }
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 50);
        }
        next_print_tick += PRINT_INTERVAL;
    }
}

// 串口中断解析
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // 立即开启下一次接收
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        
        // 协议解析机：FC [Skip] [Skip] [Skip] [LSB] [MSB]
        switch (current_state) {
            case STATE_WAIT_FC:         // Byte 1
                if (rx_byte == 0xFC) {
                    current_state = STATE_SKIP_BYTE_2;
                }
                break;

            case STATE_SKIP_BYTE_2:     // Byte 2
                current_state = STATE_SKIP_BYTE_3;
                break;

            case STATE_SKIP_BYTE_3:     // Byte 3
                current_state = STATE_SKIP_BYTE_4;
                break;

            case STATE_SKIP_BYTE_4:     // Byte 4
                current_state = STATE_READ_T_LSB;
                break;

            case STATE_READ_T_LSB:      // Byte 5 (实际数据低位)
                temp_lsb = rx_byte; 
                current_state = STATE_READ_T_MSB;
                break;

            case STATE_READ_T_MSB:      // Byte 6 (实际数据高位)
                {
                    uint8_t temp_msb = rx_byte;
                    
                    // 合成原始值 (小端: LSB在前)
                    uint16_t raw_temp = (uint16_t)temp_lsb | ((uint16_t)temp_msb << 8);
                    
                    // 校准计算: / 10.0
                    latest_temp_val = raw_temp / 10.0f;
                    
                    temp_received_flag = 1;
                    
                    current_state = STATE_WAIT_FC;
                }
                break;
                
            default:
                current_state = STATE_WAIT_FC;
                break;
        }
    }
}

// 错误处理
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

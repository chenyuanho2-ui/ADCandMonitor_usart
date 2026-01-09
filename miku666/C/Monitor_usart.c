/*
 * Monitor_usart.c
 * 严格时序记录版：250ms 固定间隔打印 + ADC 滑动窗口中值滤波
 */

#include "Monitor_usart.h"
#include "usart.h"
#include "adc.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h" // 用于排序

// 引用外部句柄
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

// ================= 配置参数 =================
#define PRINT_INTERVAL      250  // 打印间隔：严格 250ms
#define ADC_SAMPLE_INTERVAL 50   // ADC采样间隔：50ms
#define ADC_WINDOW_SIZE     5    // 中值滤波窗口大小 (5 * 50ms = 250ms 数据覆盖)

// ================= 全局变量 =================

// --- 串口协议相关 ---
#define RX_BUFFER_SIZE 1
static uint8_t rx_byte;
static uint16_t latest_temp_raw = 0; // 存储最新一次解析到的温度值
static uint8_t temp_received_flag = 0; // 标记是否收到过至少一次温度

// 协议状态机
typedef enum {
    STATE_WAIT_HEADER,
    STATE_WAIT_LEN_L,
    STATE_WAIT_LEN_H,
    STATE_WAIT_DATA,
    STATE_WAIT_XOR
} ParseState_t;

static ParseState_t current_state = STATE_WAIT_HEADER;
static uint16_t packet_len = 0;
static uint8_t packet_buf[32];
static uint8_t packet_idx = 0;

#define PROTOCOL_HEAD 0xFC
#define CMD_TEMP_UPLOAD 0x01

// --- ADC 滤波相关 ---
static uint16_t adc_window[ADC_WINDOW_SIZE] = {0}; // 滑动窗口 buffer
static uint8_t adc_win_idx = 0;                    // 当前写入位置
static uint32_t last_adc_tick = 0;

// --- 定时打印相关 ---
static uint32_t next_print_tick = 0; // 下一次打印的绝对时间目标

// ================= 内部函数 =================

// 简单的冒泡排序，用于求中值（由于数组很小，效率足够高）
static void Sort_Array(uint16_t *arr, uint8_t n) {
    uint8_t i, j;
    uint16_t temp;
    for (i = 0; i < n - 1; i++) {
        for (j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

// 获取当前窗口的中值
static uint16_t Get_Median_ADC(void) {
    uint16_t temp_buf[ADC_WINDOW_SIZE];
    
    // 1. 复制数据到临时数组（以免打乱原始滑动窗口的时序）
    // 需要临界区保护，防止复制过程中 ADC 采样中断修改数据（如果 ADC 放在中断里）
    // 但这里 ADC 采样在 Task 里，是单线程模型，不需要关中断。
    for(int i=0; i<ADC_WINDOW_SIZE; i++) {
        temp_buf[i] = adc_window[i];
    }
    
    // 2. 排序
    Sort_Array(temp_buf, ADC_WINDOW_SIZE);
    
    // 3. 取中间值
    return temp_buf[ADC_WINDOW_SIZE / 2];
}

// ================= 核心功能函数 =================

void Monitor_Init(void) {
    // 1. 启动硬件
    HAL_ADC_Start(&hadc1);
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    
    // 2. 初始化时序变量
    uint32_t now = HAL_GetTick();
    last_adc_tick = now;
    
    // 设定第一次打印的时间点（为了整齐，可以设为当前时间 + 间隔）
    next_print_tick = now + PRINT_INTERVAL;
    
    // 预填充 ADC 窗口，避免最开始数据为 0
    uint16_t initial_val = HAL_ADC_GetValue(&hadc1);
    for(int i=0; i<ADC_WINDOW_SIZE; i++) {
        adc_window[i] = initial_val;
    }
}

// 放在 main 的 while(1) 中调用
void Monitor_Task(void) {
    uint32_t current_tick = HAL_GetTick();

    // ---------------- ADC 采样任务 (每 50ms) ----------------
    if (current_tick - last_adc_tick >= ADC_SAMPLE_INTERVAL) {
        last_adc_tick = current_tick; // 更新时间戳
        
        // 采样并推入滑动窗口
        adc_window[adc_win_idx] = (uint16_t)HAL_ADC_GetValue(&hadc1);
        adc_win_idx++;
        if (adc_win_idx >= ADC_WINDOW_SIZE) {
            adc_win_idx = 0; // 环形回绕
        }
    }

    // ---------------- 打印任务 (严格 250ms 间隔) ----------------
    // 逻辑：只要当前时间 >= 目标时间，就执行打印，并把目标时间往后推 250ms
    if (current_tick >= next_print_tick) {
        
        // 检查按键是否按下 (PA3, Low Active)
        if (HAL_GPIO_ReadPin(BOTTON1_GPIO_Port, BOTTON1_Pin) == GPIO_PIN_RESET) {
            
            // 1. 获取计算好的 ADC 中值
            uint16_t median_adc = Get_Median_ADC();
            
            // 2. 准备打印
            // 注意：这里打印的是 next_print_tick，这保证了 log 里的时间是 250, 500, 750... 
            // 即使主循环有 1-2ms 的延迟，记录的时间戳依然是完美的数学间隔。
            char msg[64];
            
            if (temp_received_flag) {
                sprintf(msg, "[%u ms] T:0x%04X, ValADC:%u\r\n", 
                        next_print_tick, latest_temp_raw, median_adc);
            } else {
                // 如果还没收到过温度包，显示 Waiting
                sprintf(msg, "[%u ms] T:Wait.., ValADC:%u\r\n", 
                        next_print_tick, median_adc);
            }
            
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 50);
        }

        // 3. 设定下一次的严格时间点
        // 这里用 += 而不是 = current + 250，是为了消除累积误差。
        next_print_tick += PRINT_INTERVAL;
    }
}

// ---------------- 串口接收中断 (只负责更新 latest_temp_raw) ----------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        
        switch (current_state) {
            case STATE_WAIT_HEADER:
                if (rx_byte == PROTOCOL_HEAD) {
                    packet_buf[0] = rx_byte;
                    packet_idx = 1;
                    current_state = STATE_WAIT_LEN_L;
                }
                break;

            case STATE_WAIT_LEN_L:
                packet_buf[packet_idx++] = rx_byte;
                current_state = STATE_WAIT_LEN_H;
                break;

            case STATE_WAIT_LEN_H:
                packet_buf[packet_idx++] = rx_byte;
                packet_len = (uint16_t)packet_buf[1] | ((uint16_t)rx_byte << 8);
                if (packet_len > 32 || packet_len < 5) {
                    current_state = STATE_WAIT_HEADER;
                } else {
                    current_state = STATE_WAIT_DATA;
                }
                break;

            case STATE_WAIT_DATA:
                packet_buf[packet_idx++] = rx_byte;
                if (packet_idx >= packet_len - 1) {
                    current_state = STATE_WAIT_XOR;
                }
                break;

            case STATE_WAIT_XOR:
                packet_buf[packet_idx++] = rx_byte; 
                if (packet_idx == packet_len) {
                    uint8_t calc_xor = 0;
                    for (int i = 0; i < packet_len - 1; i++) {
                        calc_xor ^= packet_buf[i];
                    }
                    
                    if (calc_xor == rx_byte) {
                        // 校验通过
                        if (packet_buf[3] == CMD_TEMP_UPLOAD) {
                            // 更新全局温度变量 (不打印，只更新)
                            // 这样无论数据什么时候来，打印任务拿到的永远是最新的值
                            latest_temp_raw = (uint16_t)packet_buf[4] | ((uint16_t)packet_buf[5] << 8);
                            temp_received_flag = 1;
                        }
                    }
                }
                current_state = STATE_WAIT_HEADER;
                break;
                
            default:
                current_state = STATE_WAIT_HEADER;
                break;
        }
    }
}

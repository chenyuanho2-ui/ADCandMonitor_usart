/*
 * Monitor_usart.c
 * 2026-01-15 最终修正版
 * 功能：
 * 1. 协议解析：FC 0A 00 01 开头，0-100度有效范围过滤。
 * 2. ADC采样：每50ms采集一次，打印时取0.25s内的中值。
 * 3. 时序控制：
 * - 收到第一帧有效温度 -> 同步开始 (Start ADC)。
 * - 延时0.25s (等待数据积攒) -> 第一次打印 (标记为 0.00s)。
 * - 之后每0.25s打印一次。
 */

#include "Monitor_usart.h"
#include "usart.h"
#include "adc.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h" // for qsort

// 引用外部句柄
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

// ================= 宏定义与配置 =================
#define PRINT_INTERVAL_MS   250   // 打印周期 250ms
#define ADC_SAMPLE_MS       50    // ADC采样周期 50ms
#define LED_TOGGLE_MS       15000 // LED翻转周期 15s
#define TEMP_MIN            0.0f
#define TEMP_MAX            100.0f

// ADC 窗口大小 (250ms / 50ms = 5，预留多一点防止溢出)
#define MAX_ADC_SAMPLES     10    

// 协议状态机
typedef enum {
    STATE_WAIT_FC,       // 等待帧头 FC
    STATE_CHECK_LEN,     // 检查 0A
    STATE_CHECK_ZERO,    // 检查 00
    STATE_CHECK_STATUS,  // 检查 01
    STATE_READ_DATA      // 读取数据
} ProtocolState_t;

// ================= 全局变量 =================

// --- 接收与解析 ---
static uint8_t rx_byte;
static ProtocolState_t p_state = STATE_WAIT_FC;
static uint8_t data_buf[6];      
static uint8_t data_idx = 0;

// --- 数据资源 (临界区保护) ---
static volatile float g_latest_valid_temp = 0.0f; 
static volatile uint8_t g_has_valid_data = 0;     

// --- ADC 相关 ---
static uint32_t adc_values[MAX_ADC_SAMPLES];
static uint8_t adc_count = 0;
static uint32_t next_adc_tick = 0;

// --- 系统控制 ---
static uint8_t is_running = 1;              // 1:Start, 0:Stop
// static uint8_t last_btn_state;           // 已移除，避免未使用警告

// --- 时间轴 ---
static uint8_t time_synced = 0;             // 是否收到第一帧
static uint32_t time_base_tick = 0;         // 0.00s 对应的时刻
static uint32_t next_print_tick = 0;        
static uint32_t next_led_tick = 0;

// ================= 内部辅助函数 =================

// 简单的冒泡排序用于取中值 (数量很少，性能无影响)
static uint32_t Get_Median_ADC(void) {
    if (adc_count == 0) return 0;
    
    // 复制一份数据以防修改原数组 (虽然还要重置，习惯上复制更安全)
    uint32_t sorted[MAX_ADC_SAMPLES];
    uint8_t n = adc_count;
    for(int i=0; i<n; i++) sorted[i] = adc_values[i];
    
    // 冒泡排序
    for(int i=0; i<n-1; i++) {
        for(int j=0; j<n-i-1; j++) {
            if(sorted[j] > sorted[j+1]) {
                uint32_t temp = sorted[j];
                sorted[j] = sorted[j+1];
                sorted[j+1] = temp;
            }
        }
    }
    // 返回中位数
    return sorted[n/2];
}

// 更新温度 (中断调用)
static void Update_Temperature(uint8_t lsb, uint8_t msb) {
    uint16_t raw = (uint16_t)lsb | ((uint16_t)msb << 8);
    float val = raw / 10.0f;

    if (val >= TEMP_MIN && val <= TEMP_MAX) {
        g_latest_valid_temp = val;
        g_has_valid_data = 1;
        
        // 收到系统生命周期内的第一帧有效数据 -> 建立时间轴
        if (is_running && !time_synced) {
            time_synced = 1;
            uint32_t now = HAL_GetTick();
            
            // 关键逻辑：用户要求“采集到第二个温度时才算作第0s”
            // 即：第一个打印时刻标记为 0.00s。
            // 现在的时刻是 (Frame 1 Arrival)，第一次打印将在 (Frame 1 + 250ms)。
            // 所以我们将 time_base_tick 设为 (now + 250)。
            // 这样在 250ms 后打印时，(Tick - time_base) = 0。
            time_base_tick = now + PRINT_INTERVAL_MS;
            
            // 安排下一次打印和ADC采样
            next_print_tick = now + PRINT_INTERVAL_MS;
            next_adc_tick = now; // 立即开始采样ADC
        }
    }
}

// ================= 核心接口 =================

void Monitor_Init(void) {
    // 1. 启动串口接收
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    
    // 2. 初始化时间
    uint32_t now = HAL_GetTick();
    next_led_tick = now + LED_TOGGLE_MS;
    
    // 3. 提示
    char *msg = "\r\n[System Ready] Waiting for FC 0A 00 01... (1st valid frame triggers 0s start)\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

void Monitor_Task(void) {
    uint32_t now = HAL_GetTick();

    // --- 1. 按键逻辑 (PA3 / BOTTON1) ---
    // 下拉输入，按下为高电平? 
    // 原代码逻辑：if(Read == RESET) ... wait while(Read == RESET)
    // 假设按键按下是低电平(RESET)
    if (HAL_GPIO_ReadPin(BOTTON1_GPIO_Port, BOTTON1_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(20); 
        if (HAL_GPIO_ReadPin(BOTTON1_GPIO_Port, BOTTON1_Pin) == GPIO_PIN_RESET) {
            while(HAL_GPIO_ReadPin(BOTTON1_GPIO_Port, BOTTON1_Pin) == GPIO_PIN_RESET); // 等待松开
            
            is_running = !is_running;
            
            if (is_running) {
                // 重启：清除同步标志，等待新数据重建时间轴
                time_synced = 0; 
                adc_count = 0;
                char *s = "-> START\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 50);
            } else {
                char *s = "-> STOP\r\n";
                HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 50);
            }
        }
    }

    // --- 2. LED 15s 翻转 ---
    if (now >= next_led_tick) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        next_led_tick = now + LED_TOGGLE_MS;
    }
    
    // 只有在运行且已同步(收到过第一帧)后，才执行ADC和打印
    if (is_running && time_synced) {
        
        // --- 3. ADC 采样 (每50ms) ---
        if (now >= next_adc_tick) {
            // 启动一次转换
            HAL_ADC_Start(&hadc1);
            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                uint32_t val = HAL_ADC_GetValue(&hadc1);
                
                // 存入缓冲
                if (adc_count < MAX_ADC_SAMPLES) {
                    adc_values[adc_count++] = val;
                }
            }
            // 设定下次采样时间
            next_adc_tick += ADC_SAMPLE_MS;
            if (next_adc_tick < now) next_adc_tick = now + ADC_SAMPLE_MS;
        }

        // --- 4. 打印逻辑 (每250ms) ---
        if (now >= next_print_tick) {
            
            // a. 获取温度 (原子操作)
            float current_temp = 0.0f;
            uint8_t has_data = 0;
            __disable_irq();
            current_temp = g_latest_valid_temp;
            has_data = g_has_valid_data;
            __enable_irq();

            if (has_data) {
                // b. 获取ADC中值
                uint32_t median_adc = Get_Median_ADC();
                
                // c. 计算相对时间 (注意 time_base_tick 已经是 FirstFrameTime + 250ms)
                // 这样第一次打印时 (now - time_base_tick) ≈ 0
                // 加上 0.005f 是为了四舍五入显示更加友好，防止出现 -0.00
                float relative_time = (int32_t)(now - time_base_tick) / 1000.0f; 
                
                // d. 打印
                char msg[64];
                // 格式: [时间s] T:温度 C, ADC:值
                sprintf(msg, "[%.2fs] T:%.1f C, ADC:%lu\r\n", 
                        relative_time, current_temp, median_adc);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 50);
                
                // e. 清空ADC缓冲，准备下一个0.25s周期
                adc_count = 0;
            }

            // f. 设定下次打印
            next_print_tick += PRINT_INTERVAL_MS;
            if (next_print_tick < now) next_print_tick = now + PRINT_INTERVAL_MS;
        }
    }
}

// 串口中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
        
        // FC 0A 00 01 [Byte5 Byte6] ...
        switch (p_state) {
            case STATE_WAIT_FC:
                if (rx_byte == 0xFC) p_state = STATE_CHECK_LEN;
                break;

            case STATE_CHECK_LEN: // 0A
                if (rx_byte == 0x0A) p_state = STATE_CHECK_ZERO;
                else if (rx_byte == 0x05) p_state = STATE_WAIT_FC; // 忽略请求帧
                else p_state = STATE_WAIT_FC;
                break;

            case STATE_CHECK_ZERO: // 00
                if (rx_byte == 0x00) p_state = STATE_CHECK_STATUS;
                else p_state = STATE_WAIT_FC;
                break;

            case STATE_CHECK_STATUS: // 01
                if (rx_byte == 0x01) {
                    p_state = STATE_READ_DATA;
                    data_idx = 0;
                }
                else p_state = STATE_WAIT_FC;
                break;

            case STATE_READ_DATA:
                data_buf[data_idx++] = rx_byte;
                if (data_idx >= 6) {
                    // data_buf[0]=LSB, data_buf[1]=MSB
                    Update_Temperature(data_buf[0], data_buf[1]);
                    p_state = STATE_WAIT_FC;
                }
                break;
                
            default:
                p_state = STATE_WAIT_FC;
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

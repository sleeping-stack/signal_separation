#ifndef SIGNAL_SEPARATION_ADC_DMA_TIMER_H
#define SIGNAL_SEPARATION_ADC_DMA_TIMER_H

#include "adc.h"
#include "tim.h"
#include "main.h"
#include <stdio.h>
#include "dma.h"

#define ADC_DATA_LENGTH 4096 // 定义采集数据长度

void adc_timer_init();
void adc_start_one_time();
int32_t set_ADC_Sampling_Rate(uint32_t target_fs);

extern uint8_t g_adc1_dma_complete_flag; // adc1 数据 dma 采集完成标志
extern uint32_t g_adc_sample_rate; // 当前ADC采样率（Sps）
extern uint16_t g_adc1_dma_data[ADC_DATA_LENGTH]; // adc1 dma 采集数据缓冲区

#endif // SIGNAL_SEPARATION_ADC_DMA_TIMER_H

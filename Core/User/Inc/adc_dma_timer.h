#ifndef SIGNAL_SEPARATION_ADC_DMA_TIMER_H
#define SIGNAL_SEPARATION_ADC_DMA_TIMER_H

#include "adc.h"
#include "tim.h"
#include "main.h"
#include <stdio.h>

#define ADC_DATA_LENGTH 4096    //定义采集数据长度
#define ADC_DMA_BUFF_ADDR 0x30000000
#define g_adc1_dma_data (*(uint16_t (*)[ADC_DATA_LENGTH])ADC_DMA_BUFF_ADDR)

void adc_timer_init();
void adc_start_one_time();
int32_t set_ADC_Sampling_Rate(uint32_t target_fs);

extern uint8_t g_adc1_dma_complete_flag; //adc1 数据 dma 采集完成标志
extern uint32_t g_adc_sample_rate; // 当前ADC采样率（Sps）

#endif //SIGNAL_SEPARATION_ADC_DMA_TIMER_H

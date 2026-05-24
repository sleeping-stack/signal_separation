#ifndef SIGNAL_SEPARATION_FFT_H
#define SIGNAL_SEPARATION_FFT_H

#include "arm_math.h"
#include  "adc_dma_timer.h"
#include  <stdio.h>

#define FFT_LENGTH  ADC_DATA_LENGTH // FFT长度与采集数据长度相同
// 设定一个噪声阈值(mV)，低于此幅值的峰值不参与分析，防止误判噪声
#define NOISE_THRESHOLD_MV 20.0f
#define PEAK_COUNT_MAX 20

// 定义波形类型
typedef enum {
    WAVE_UNKNOWN = 0,
    WAVE_SINE,
    WAVE_TRIANGLE
} WaveType_e;

// 信号信息结构体
typedef struct {
    float32_t frequency; // 频率 (Hz)
    float32_t amplitude; // 幅值 (mV)
    WaveType_e type; // 波形类型
} SignalInfo_t;

extern SignalInfo_t sig_A, sig_B; // 全局变量存储A'，B'分析结果

void perform_fft();

SignalInfo_t interpolate_specific_peak(const float32_t *fft_mag, uint16_t peak_idx);

void analyze_mixed_signals(const float32_t *fft_mag,
                           SignalInfo_t *sig1, SignalInfo_t *sig2);

void test_signal_analysis(void);

#endif //SIGNAL_SEPARATION_FFT_H

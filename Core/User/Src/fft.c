#include "fft.h"

float32_t fft_inputbuf[2 * FFT_LENGTH] = {0}; // 转换为浮点数的输入信号
float32_t fft_outputbuf[FFT_LENGTH] = {0};
/* FFT 实例与状态标志 */
arm_cfft_instance_f32 scfft;

void perform_fft() {
    // 提取直流分量
    float32_t dc_component = 0.0f;
    for (size_t i = 0; i < FFT_LENGTH; i++) {
        dc_component += g_adc1_dma_data[i];
    }
    dc_component /= FFT_LENGTH;

    // 使用Hanning窗提高fft精度
    float32_t hanning_window[FFT_LENGTH];
    Hanningwindow(hanning_window);

    // 生成信号序列
    for (size_t i = 0; i < FFT_LENGTH; i++) {
        const float32_t temp =
                (float) ((g_adc1_dma_data[i] - dc_component) * 3300.0 / 65535.0); // 先去除直流分量再转换为电压值
        fft_inputbuf[2 * i] = temp * hanning_window[i]; // 实部，单位为mV
        fft_inputbuf[2 * i + 1] = 0.0f; // 虚部
    }

    arm_cfft_init_f32(&scfft, FFT_LENGTH); // 初始化scfft结构体,设定FFT参数
    arm_cfft_f32(&scfft, fft_inputbuf, 0, 1); // FFT计算
    arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf,
                      FFT_LENGTH); // 把运算结果复数求模得幅值

    // 对非直流分量进行频谱归一化（单边谱）
    for (size_t i = 1; i < FFT_LENGTH / 2; i++) {
        fft_outputbuf[i] = fft_outputbuf[i] / ((float)FFT_LENGTH / 2);
    }
    // 直流分量单独归一化
    fft_outputbuf[0] = fft_outputbuf[0] / FFT_LENGTH;

    // 抹除右半部分
    for (size_t i = FFT_LENGTH / 2; i < FFT_LENGTH; i++) {
        fft_outputbuf[i] = 0.0f;
    }
}

/**
 * @brief 应用汉宁窗函数到输入数组
 *
 * 该函数计算并应用汉宁窗，用于减少FFT分析中的频谱泄漏
 * 汉宁窗公式为: w(n) = 0.5 * (1 - cos(2πn/N)), 其中n是样本索引，N是FFT长度
 *
 * @param hanning_window
 * 指向存储汉宁窗系数的浮点数组的指针，数组长度为FFT_LENGTH
 * @return 无返回值
 */
void Hanningwindow(float32_t hanning_window[]) // 汉宁窗
{
    for (size_t i = 0; i < FFT_LENGTH; i++) {
        hanning_window[i] = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * (float)i / FFT_LENGTH));
    }
}

#include "fft.h"

float32_t fft_inputbuf[2 * FFT_LENGTH] __attribute__((section(".FFT_BUF_8192"))); // 转换为浮点数的输入信号
float32_t fft_outputbuf[FFT_LENGTH] __attribute__((section(".FFT_BUF_4096"))); // FFT 变换后的复数结果
float32_t hanning_window[FFT_LENGTH]
    __attribute__((section(".FFT_BUF_4096"))); // Hanning窗系数 (复用4096段，在fft_outputbuf之后)
/* FFT 实例与状态标志 */
arm_cfft_instance_f32 scfft;

SignalInfo_t sig_A, sig_B = {0.0f, 0.0f, WAVE_UNKNOWN};

void perform_fft()
{
    memset(fft_inputbuf, 0, sizeof(fft_inputbuf));
    memset(fft_outputbuf, 0, sizeof(fft_outputbuf));

    // 提取直流分量
    uint32_t dc_sum = 0;
    for (size_t i = 0; i < FFT_LENGTH; i++)
    {
        dc_sum += g_adc1_dma_data[i];
    }
    const float32_t dc_component = (float32_t)dc_sum / FFT_LENGTH;

    // 使用Hanning窗提高fft精度 (hanning_window 已定义为全局缓冲区，避免栈溢出)
    arm_hanning_f32(hanning_window, FFT_LENGTH);

    // 生成信号序列
    for (size_t i = 0; i < FFT_LENGTH; i++)
    {
        const float32_t temp =
            ((float32_t)g_adc1_dma_data[i] - dc_component) * 3300.0f / 65535.0f; // 先去除直流分量再转换为电压值
        fft_inputbuf[2 * i] = temp * hanning_window[i]; // 实部，单位为mV
        fft_inputbuf[2 * i + 1] = 0.0f; // 虚部
    }

    arm_cfft_init_f32(&scfft, FFT_LENGTH); // 初始化scfft结构体,设定FFT参数
    arm_cfft_f32(&scfft, fft_inputbuf, 0, 1); // FFT计算
    arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf,
                      FFT_LENGTH); // 把运算结果复数求模得幅值

    // 原始单边谱除以 (N/2)，加窗补偿乘 2.0，合并计算就是乘 4.0 / N
    for (size_t i = 1; i < FFT_LENGTH / 2; i++)
    {
        fft_outputbuf[i] = fft_outputbuf[i] * 4.0f / (float32_t)FFT_LENGTH;
    }
    // 直流分量单独归一化
    fft_outputbuf[0] = fft_outputbuf[0] * 2.0f / (float32_t)FFT_LENGTH;

    // 抹除右半部分
    for (uint16_t i = FFT_LENGTH / 2; i < FFT_LENGTH; i++)
    {
        fft_outputbuf[i] = 0.0f;
    }
}

/**
 * @brief 对指定的 FFT 峰值下标进行双峰插值
 */
SignalInfo_t interpolate_specific_peak(const float32_t *fft_mag, const uint16_t peak_idx)
{
    SignalInfo_t peak = {0.0f, 0.0f, WAVE_UNKNOWN};

    // 边界保护
    if (peak_idx == 0 || peak_idx >= FFT_LENGTH / 2 - 1)
        return peak;

    const float32_t y0 = fft_mag[peak_idx];
    const float32_t y_minus = fft_mag[peak_idx - 1];
    const float32_t y_plus = fft_mag[peak_idx + 1];

    float32_t y1 = 0.0f;
    int direction = 0;

    if (y_plus > y_minus)
    {
        y1 = y_plus;
        direction = 1;
    }
    else
    {
        y1 = y_minus;
        direction = -1;
    }

    float32_t delta = 0.0f;
    if (y0 > 0.0f)
    {
        const float32_t R = y1 / y0;
        delta = (2.0f * R - 1.0f) / (R + 1.0f);
        if (delta < 0.0f)
            delta = 0.0f;
        if (delta > 0.5f)
            delta = 0.5f;
    }

    // 计算高精度频率与幅值
    float32_t k_true = (float32_t)peak_idx + (float32_t)direction * delta;
    peak.frequency = k_true * ((float32_t)g_adc_sample_rate / (float32_t)FFT_LENGTH);

    float32_t pi_delta = PI * delta;
    peak.amplitude = y0 * (pi_delta * (1.0f - delta * delta)) / sinf(pi_delta);

    return peak;
}

/**
 * @brief 分析混合信号，提取前两大主频信号的频率、幅值和波形类型
 * @param fft_mag  求模并归一化后的 FFT 结果数组
 * @param sig1     输出指针，存储第一大信号特征
 * @param sig2     输出指针，存储第二大信号特征
 */
void analyze_mixed_signals(const float32_t *fft_mag, SignalInfo_t *sig1, SignalInfo_t *sig2)
{
    uint16_t local_peaks[PEAK_COUNT_MAX] = {0}; // 暂存局部峰值的下标
    uint16_t peak_count = 0;

    // 1. 寻找所有的“局部最大值”（即比左右两边都大的点），跳过直流(0,1)
    for (size_t i = 2; i < FFT_LENGTH / 2 - 1; i++)
    {
        if (fft_mag[i] > fft_mag[i - 1] && fft_mag[i] > fft_mag[i + 1] && fft_mag[i] > NOISE_THRESHOLD_MV)
        {
            if (peak_count < PEAK_COUNT_MAX)
            {
                local_peaks[peak_count++] = i;
            }
        }
    }

    // 2. 在局部峰值中找出最大的两个（即两个叠加信号的基频）
    uint16_t top1_idx = 0, top2_idx = 0;
    float32_t max1 = 0.0f, max2 = 0.0f;

    for (size_t i = 0; i < peak_count; i++)
    {
        float32_t val = fft_mag[local_peaks[i]];
        if (val > max1)
        {
            max2 = max1;
            top2_idx = top1_idx;
            max1 = val;
            top1_idx = local_peaks[i];
        }
        else if (val > max2)
        {
            max2 = val;
            top2_idx = local_peaks[i];
        }
    }

    // 3. 对找到的两个主峰进行高精度插值
    *sig1 = interpolate_specific_peak(fft_mag, top1_idx);
    *sig2 = interpolate_specific_peak(fft_mag, top2_idx);

    // 4. 判别波形类型 (验证三次谐波)
    SignalInfo_t *target_sigs[2] = {sig1, sig2};

    for (size_t i = 0; i < 2; i++)
    {
        SignalInfo_t *sig = target_sigs[i];

        // 判断是否为有效波形
        if (sig->amplitude < NOISE_THRESHOLD_MV)
        {
            sig->type = WAVE_UNKNOWN;
            continue;
        }

        // 理论上的三次谐波频率
        float32_t target_3rd_freq = sig->frequency * 3.0f;

        // 奈奎斯特限制：如果三次谐波超出了采样率的一半，我们就看不见它了，默认归为正弦波
        if (target_3rd_freq >= (float32_t)g_adc_sample_rate / 2.0f)
        {
            sig->type = WAVE_SINE;
            continue;
        }

        // 估算三次谐波在 FFT 数组中的粗略位置
        const uint16_t bin_3rd = (uint16_t)(target_3rd_freq * FFT_LENGTH / (float32_t)g_adc_sample_rate + 0.5f);

        // 在三次谐波附近寻找最大值（允许 ±2 bin 的频偏误差）
        float32_t max_3rd_amp = 0.0f;
        for (int16_t j = -2; j <= 2; j++)
        {
            int search_idx = bin_3rd + j;
            if (search_idx > 0 && search_idx < FFT_LENGTH / 2)
            {
                if (fft_mag[search_idx] > max_3rd_amp)
                {
                    max_3rd_amp = fft_mag[search_idx];
                }
            }
        }

        // 计算 3次谐波 与 基频 的幅值比
        const float32_t ratio = max_3rd_amp / sig->amplitude;

        // 理论上三角波的三次谐波幅度是 1/9 (约 0.111)
        // 考虑到硬件低通滤波衰减和窗函数泄露，设置宽容度 [0.05, 0.20]
        if (ratio >= 0.05f && ratio <= 0.20f)
        {
            sig->type = WAVE_TRIANGLE;
        }
        else
        {
            sig->type = WAVE_SINE; // 找不到合格的三次谐波，即为正弦波
        }
    }
}

void test_signal_analysis(void)
{
    // 1. FFT 计算（包含加汉宁窗和幅值补偿）
    perform_fft();

    // 2. 定义存储结果的结构体
    SignalInfo_t sig1, sig2;

    // 3. 提取频率和波形 (假设采样率为 10000 Hz)
    analyze_mixed_signals(fft_outputbuf, &sig1, &sig2);

    if (sig1.frequency <= sig2.frequency)
    {
        sig_A = sig1;
        sig_B = sig2;
    }
    else
    {
        sig_A = sig2;
        sig_B = sig1;
    }

    // 将A、B信号频率取整到最近的5000Hz整数倍
    sig_A.frequency = roundf(sig_A.frequency / FREQ_STEP_HZ) * FREQ_STEP_HZ;
    sig_B.frequency = roundf(sig_B.frequency / FREQ_STEP_HZ) * FREQ_STEP_HZ;

    if (sig_A.type == WAVE_TRIANGLE)
    {
        sig_A.amplitude *= PI * PI / 8.0f;
    }
    if (sig_B.type == WAVE_TRIANGLE)
    {
        sig_B.amplitude *= PI * PI / 8.0f;
    }
}

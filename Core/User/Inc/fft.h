#ifndef SIGNAL_SEPARATION_FFT_H
#define SIGNAL_SEPARATION_FFT_H

#include "arm_math.h"
#include  "adc_dma_timer.h"

#define FFT_LENGTH  4096
extern float32_t fft_outputbuf[FFT_LENGTH];

void perform_fft();
void Hanningwindow(float32_t hanning_window[]);

#endif //SIGNAL_SEPARATION_FFT_H

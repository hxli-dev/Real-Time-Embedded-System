// fft_processing.h

#pragma once
#include "globals.h"

void fft_init();
void fft_mag(const float *timeData, float *magOut);
void remove_dc(float *data);

void compute_fft(int start_idx);

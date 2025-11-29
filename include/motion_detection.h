// motion_detection.h

#pragma once
#include "globals.h"

static constexpr float ACC_TREMOR_TH = 500.0f;
static constexpr float GYRO_TREMOR_TH = 1000.0f;
static constexpr float ACC_DYS_TH = 10000.0f;
static constexpr float GYRO_DYS_TH = 5000.0f;

float band_energy(const float *mag, float freq_low, float freq_high);
float dominant_frequency();

void print_result();

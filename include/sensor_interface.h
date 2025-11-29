// sensor_interface.h

#pragma once
#include "globals.h"

void write_register(uint8_t reg, uint8_t value);
uint8_t read_register(uint8_t reg);
int16_t read_16bit_value(uint8_t low_reg, uint8_t high_reg);

static constexpr float ACC_SENSITIVITY = 0.061f * 1e-3f;
static constexpr float GYRO_SENSITIVITY = 8.75f * 1e-3f;

float *get_acc_g();
float *get_gyro_g();
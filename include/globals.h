// globals.h

#ifndef GLOBALS_H
#define GLOBALS_H

#include "mbed.h"
#include "arm_math.h"

extern BufferedSerial serial_port;
FileHandle *mbed_override_console(int);

extern I2C i2c;

#define LSM6DSL_ADDR (0x6A << 1)
#define WHO_AM_I 0x0F
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D
#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27

static constexpr int BUFFER_SIZE = 512;
static constexpr int BATCH_TIME_MS = 3000;
static constexpr int SAMPLE_INTERVAL_US = BATCH_TIME_MS * 1000 / BUFFER_SIZE;

extern arm_rfft_fast_instance_f32 fftInst;
extern float fftOut[BUFFER_SIZE];

extern float accX_time[BUFFER_SIZE];
extern float accY_time[BUFFER_SIZE];
extern float accZ_time[BUFFER_SIZE];
extern float gyroX_time[BUFFER_SIZE];
extern float gyroY_time[BUFFER_SIZE];
extern float gyroZ_time[BUFFER_SIZE];

extern float magAccX[BUFFER_SIZE / 2];
extern float magAccY[BUFFER_SIZE / 2];
extern float magAccZ[BUFFER_SIZE / 2];
extern float magGyroX[BUFFER_SIZE / 2];
extern float magGyroY[BUFFER_SIZE / 2];
extern float magGyroZ[BUFFER_SIZE / 2];

extern float accX_g[BUFFER_SIZE];
extern float accY_g[BUFFER_SIZE];
extern float accZ_g[BUFFER_SIZE];
extern float gyroX_g[BUFFER_SIZE];
extern float gyroY_g[BUFFER_SIZE];
extern float gyroZ_g[BUFFER_SIZE];

extern int circular_idx;

extern DigitalOut tremorLed;
extern DigitalOut dyskinesiaLed;

extern volatile bool batchReady;
extern volatile bool isReading;
extern Ticker sampleTicker;
extern EventQueue queue;
extern Thread worker;
extern Ticker tremorBlinkTicker;
extern Ticker dyskinesiaBlinkTicker;

#endif // GLOBALS_H

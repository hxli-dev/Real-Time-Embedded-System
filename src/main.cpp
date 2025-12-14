#include "mbed.h"
#include "arm_math.h"
#include "globals.h"
#include "sensor_interface.h"
#include "fft_processing.h"
#include "motion_detection.h"
#include "ble_rtes.h"


BufferedSerial serial_port(USBTX, USBRX, 115200);
I2C i2c(PB_11, PB_10);

Ticker sampleTicker;
EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread worker;

DigitalOut tremorLed(LED3);
DigitalOut dyskinesiaLed(LED1);
DigitalOut freezeLed(LED2);

volatile bool isReading = false;
volatile bool batchReady = false;

arm_rfft_fast_instance_f32 fftInst;
float fftOut[BUFFER_SIZE];

int circular_idx = 0;

float accX_time[BUFFER_SIZE];
float accY_time[BUFFER_SIZE];
float accZ_time[BUFFER_SIZE];
float gyroX_time[BUFFER_SIZE];
float gyroY_time[BUFFER_SIZE];
float gyroZ_time[BUFFER_SIZE];

float magAccX[BUFFER_SIZE / 2];
float magAccY[BUFFER_SIZE / 2];
float magAccZ[BUFFER_SIZE / 2];
float magGyroX[BUFFER_SIZE / 2];
float magGyroY[BUFFER_SIZE / 2];
float magGyroZ[BUFFER_SIZE / 2];

float accX_g[BUFFER_SIZE];
float accY_g[BUFFER_SIZE];
float accZ_g[BUFFER_SIZE];
float gyroX_g[BUFFER_SIZE];
float gyroY_g[BUFFER_SIZE];
float gyroZ_g[BUFFER_SIZE];

Ticker tremorBlinkTicker;
Ticker dyskinesiaBlinkTicker;

Ticker freezeBlinkTicker;

float currentTremorEnergy = 0.0f;
float currentDyskinesiaEnergy = 0.0f;
float currentFreezeIndex = 0.0f;
bool isFreezing = false;


FileHandle *mbed::mbed_override_console(int)
{
    return &serial_port;
}

void write_register(uint8_t reg, uint8_t value)
{
    char data[2] = {(char)reg, (char)value};
    i2c.write(LSM6DSL_ADDR, data, 2);
}

uint8_t read_register(uint8_t reg)
{
    char byte = reg;
    i2c.write(LSM6DSL_ADDR, &byte, 1, true);
    i2c.read(LSM6DSL_ADDR, &byte, 1);
    return (uint8_t)byte;
}

int16_t read_16bit_value(uint8_t low_reg, uint8_t high_reg)
{
    uint8_t low = read_register(low_reg);
    uint8_t high = read_register(high_reg);
    return (int16_t)((high << 8) | low);
}

void blink_tremor_led()
{
    tremorLed = !tremorLed;
}

void blink_dyskinesia_led()
{
    dyskinesiaLed = !dyskinesiaLed;
}

void blink_freeze_led()
{
    freezeLed = !freezeLed;
}


void update_led_blinking()
{

    float tremor_interval_ms = 500.0f - currentTremorEnergy * 300.0f;
    tremor_interval_ms = std::max(100.0f, std::min(tremor_interval_ms, 500.0f));

    if (currentTremorEnergy > 0.0f)
    {
        tremorBlinkTicker.attach(
            blink_tremor_led,
            std::chrono::milliseconds((int)tremor_interval_ms));
    }
    else
    {
        tremorBlinkTicker.detach();
        tremorLed = 0;
    }


    float dyskinesia_interval_ms = 500.0f - powf(currentDyskinesiaEnergy, 1.5f) * 600.0f;
    dyskinesia_interval_ms = std::max(100.0f, std::min(dyskinesia_interval_ms, 500.0f));

    if (currentDyskinesiaEnergy > 0.0f)
    {
        dyskinesiaBlinkTicker.attach(
            blink_dyskinesia_led,
            std::chrono::milliseconds((int)dyskinesia_interval_ms));
    }
    else
    {
        dyskinesiaBlinkTicker.detach();
        dyskinesiaLed = 0;
    }


    if (currentFreezeIndex < 1.0f)
    {
        freezeBlinkTicker.detach();
        freezeLed = 0;
    }
    else if (currentFreezeIndex <= 2.0f)
    {

        freezeBlinkTicker.attach(
            blink_freeze_led,
            std::chrono::milliseconds(700));
    }
    else
    {
   
        freezeBlinkTicker.attach(
            blink_freeze_led,
            std::chrono::milliseconds(200));
    }
}


float *get_acc_g()
{
    static float acc[3];
    acc[0] = read_16bit_value(OUTX_L_XL, OUTX_H_XL) * ACC_SENSITIVITY;
    acc[1] = read_16bit_value(OUTY_L_XL, OUTY_H_XL) * ACC_SENSITIVITY;
    acc[2] = read_16bit_value(OUTZ_L_XL, OUTZ_H_XL) * ACC_SENSITIVITY;
    return acc;
}

float *get_gyro_g()
{
    static float gyro[3];
    gyro[0] = read_16bit_value(OUTX_L_G, OUTX_H_G) * GYRO_SENSITIVITY;
    gyro[1] = read_16bit_value(OUTY_L_G, OUTY_H_G) * GYRO_SENSITIVITY;
    gyro[2] = read_16bit_value(OUTZ_L_G, OUTZ_H_G) * GYRO_SENSITIVITY;
    return gyro;
}

void fft_init()
{
    arm_rfft_fast_init_f32(&fftInst, BUFFER_SIZE);
}

void fft_mag(const float *timeData, float *mag)
{
    arm_rfft_fast_f32(&fftInst, (float *)timeData, fftOut, 0);
    for (int k = 0; k < BUFFER_SIZE / 2; ++k)
    {
        float re = fftOut[2 * k];
        float im = fftOut[2 * k + 1];
        mag[k] = sqrtf(re * re + im * im);
    }
}

void remove_dc(float *data)
{
    float sum = 0.0f;
    for (int i = 0; i < BUFFER_SIZE; i++)
        sum += data[i];
    float mean = sum / BUFFER_SIZE;
    for (int i = 0; i < BUFFER_SIZE; i++)
        data[i] -= mean;
}

void compute_fft(int start_idx)
{
    for (int i = 0; i < BUFFER_SIZE; ++i)
    {
        int j = (circular_idx + i) % BUFFER_SIZE;
        accX_time[i] = accX_g[j];
        accY_time[i] = accY_g[j];
        accZ_time[i] = accZ_g[j];
        gyroX_time[i] = gyroX_g[j];
        gyroY_time[i] = gyroY_g[j];
        gyroZ_time[i] = gyroZ_g[j];
    }

    remove_dc(accX_time);
    remove_dc(accY_time);
    remove_dc(accZ_time);
    remove_dc(gyroX_time);
    remove_dc(gyroY_time);
    remove_dc(gyroZ_time);

    fft_mag(accX_time, magAccX);
    fft_mag(accY_time, magAccY);
    fft_mag(accZ_time, magAccZ);
    fft_mag(gyroX_time, magGyroX);
    fft_mag(gyroY_time, magGyroY);
    fft_mag(gyroZ_time, magGyroZ);
}

float band_energy(const float *mag, float freq_low, float freq_high)
{
    float sampling_freq = 1.0f / (SAMPLE_INTERVAL_US * 1e-6f);
    float freq_resolution = sampling_freq / BUFFER_SIZE;
    float total_energy = 0.0f;
    int bin_start = static_cast<int>(ceilf(freq_low / freq_resolution));
    int bin_end = static_cast<int>(floorf(freq_high / freq_resolution));
    bin_start = max(0, min(bin_start, BUFFER_SIZE / 2 - 1));
    bin_end = max(0, min(bin_end, BUFFER_SIZE / 2 - 1));

    for (int bin = bin_start; bin <= bin_end; ++bin)
        total_energy += mag[bin] * mag[bin];
    return total_energy;
}


float compute_freeze_index()
{
    // Total acceleration magnitude spectrum
    static float magAccTotal[BUFFER_SIZE / 2];

    for (int i = 0; i < BUFFER_SIZE/2; i++)
        magAccTotal[i] = magAccX[i] + magAccY[i] + magAccZ[i];

    // Freezing band: 3–8 Hz (freeze band)
    float freeze_band = band_energy(magAccTotal, 3.0f, 8.0f);

    // Locomotor band: 0.5–3 Hz
    float loco_band = band_energy(magAccTotal, 0.5f, 3.0f);

    if (loco_band < 1e-6f)  // avoid division by zero
        return 0.0f;

    return freeze_band / loco_band;
}

float dominant_frequency()
{
    float sampling_period_s = SAMPLE_INTERVAL_US * 1e-6f;
    float sampling_freq = 1.0f / sampling_period_s;
    float freq_resolution = sampling_freq / BUFFER_SIZE;

    float combined_mag[BUFFER_SIZE / 2] = {0};
    for (int bin = 0; bin < BUFFER_SIZE / 2; bin++)
    {
        combined_mag[bin] = magAccX[bin] + magAccY[bin] + magAccZ[bin] +
                            magGyroX[bin] + magGyroY[bin] + magGyroZ[bin];
    }

    int bin_start = static_cast<int>(1.0f / freq_resolution);
    int bin_end = static_cast<int>(10.0f / freq_resolution);

    float max_magnitude = combined_mag[bin_start];
    int dominant_bin = bin_start;

    for (int bin = bin_start + 1; bin <= bin_end; bin++)
    {
        if (combined_mag[bin] > max_magnitude)
        {
            max_magnitude = combined_mag[bin];
            dominant_bin = bin;
        }
    }

    return dominant_bin * freq_resolution;
}

void print_result()
{
    float E5_7_acc = band_energy(magAccX, 5.0f, 7.0f) +
                     band_energy(magAccY, 5.0f, 7.0f) +
                     band_energy(magAccZ, 5.0f, 7.0f);
    float E5_7_gyro = band_energy(magGyroX, 5.0f, 7.0f) +
                      band_energy(magGyroY, 5.0f, 7.0f) +
                      band_energy(magGyroZ, 5.0f, 7.0f);

    float E3_5_acc = band_energy(magAccX, 3.0f, 5.0f) +
                     band_energy(magAccY, 3.0f, 5.0f) +
                     band_energy(magAccZ, 3.0f, 5.0f);
    float E3_5_gyro = band_energy(magGyroX, 3.0f, 5.0f) +
                      band_energy(magGyroY, 3.0f, 5.0f) +
                      band_energy(magGyroZ, 3.0f, 5.0f);

    float dom_freq = dominant_frequency();

    bool isDyskinesia = (E5_7_acc > ACC_DYS_TH) && (E5_7_gyro > GYRO_DYS_TH) &&
                        (dom_freq >= 5.0f && dom_freq <= 7.0f);
    bool isTremor = (E3_5_acc > ACC_TREMOR_TH) && (E3_5_gyro > GYRO_TREMOR_TH) &&
                (dom_freq >= 3.0f && dom_freq <= 5.0f);


    float freezeIndex = compute_freeze_index();
currentFreezeIndex = freezeIndex;

// Threshold typically ≈ 2.0
isFreezing = (freezeIndex > 2.0f);

printf("Freeze Index (FI) = %.3f\r\n", freezeIndex);
printf("Is Freezing of Gait Detected: %s\r\n", isFreezing ? "true" : "false");

    printf("Is the accelerometer and the gyroscope energy higher than the threshold value in the 5-7 Hz band: %s\r\n", isDyskinesia ? "true" : "false");
    printf("Is the accelerometer and the gyroscope energy higher than the threshold value in the 3-5 Hz band: %s\r\n", isTremor ? "true" : "false");
    printf("Dominant frequency: %.2f Hz\r\n", dom_freq);

    currentTremorEnergy = isTremor ? E3_5_acc + E3_5_gyro : 0.0f;
    currentDyskinesiaEnergy = isDyskinesia ? E5_7_acc + E5_7_gyro : 0.0f;

    update_led_blinking();

    printf("THIS BATCH: Tremor: %s, Dyskinesia: %s, Freezing: %s\r\n",
       isTremor ? "Yes" : "No",
       isDyskinesia ? "Yes" : "No",
       isFreezing ? "Yes" : "No");

    printf("DATA,%.3f,%.3f,%.3f,%d,%d,%d\r\n",
           currentTremorEnergy,
           currentDyskinesiaEnergy,
           currentFreezeIndex,
           isTremor ? 1 : 0,
           isDyskinesia ? 1 : 0,
           isFreezing ? 1 : 0);

    rtes_ble::update(isTremor, isDyskinesia, isFreezing,
                 currentTremorEnergy, currentDyskinesiaEnergy, currentFreezeIndex);

}

void sample_isr()
{
    queue.call([]
               {
        float *acc = get_acc_g();
        accX_g[circular_idx] = acc[0];
        accY_g[circular_idx] = acc[1];
        accZ_g[circular_idx] = acc[2];
        float *gyro = get_gyro_g();
        gyroX_g[circular_idx] = gyro[0];
        gyroY_g[circular_idx] = gyro[1];
        gyroZ_g[circular_idx] = gyro[2];
        circular_idx = (circular_idx + 1) % BUFFER_SIZE;

        static int batchCount = 0;
        if (++batchCount >= BUFFER_SIZE) {
            batchReady = true;
            batchCount = 0;
        } });
}

int main()
{
    dyskinesiaLed = 0;
    tremorLed = 0;

    fft_init();
    worker.start(callback(&queue, &EventQueue::dispatch_forever));
    rtes_ble::start();


    i2c.frequency(400000);

    uint8_t id = read_register(WHO_AM_I);
    printf("WHO_AM_I = 0x%02X\r\n", id);
    if (id != 0x6A)
    {
        printf("Sensor not found!\r\n");
        while (1)
            ;
    }

    write_register(CTRL1_XL, 0x40);
    write_register(CTRL2_G, 0x40);
    printf("Sensor configured\r\n");

    isReading = true;

    while (true)
    {
        sampleTicker.attach(sample_isr, std::chrono::microseconds(SAMPLE_INTERVAL_US));

        if (batchReady)
        {
            tremorLed = 0;
            dyskinesiaLed = 0;
            compute_fft(circular_idx);
            print_result();
            batchReady = false;
            printf("\nSampling data for 3 seconds...\r\n");
        }

        ThisThread::sleep_for(1000ms);
    }
}

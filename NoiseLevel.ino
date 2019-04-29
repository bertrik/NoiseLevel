/** 
 * ESP32 I2S Noise Level Example.
 * 
 * This example calculates a mean noise level.
 * This example is Public Domain.
 * 
 * @author maspetsberger 
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"

// size of noise sample
#define SAMPLES 1024

const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = SAMPLES;

#define OCTAVES 9

// our FFT data
static float real[SAMPLES];
static float imag[SAMPLES];
static arduinoFFT fft(real, imag, SAMPLES, SAMPLES);
static float energy[OCTAVES];

static void print(const char *fmt, ...)
{
    // format it
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // send it to serial
    Serial.write(buf);
}

static void integerToFloat(int32_t * integer, float *vReal, float *vImag, uint16_t samples)
{
    for (uint16_t i = 0; i < samples; i++) {
        vReal[i] = integer[i] / 65536;
        vImag[i] = 0.0;
    }
}

static void calculateEnergy(float *vReal, float *vImag, uint16_t samples)
{
    for (uint16_t i = 0; i < samples; i++) {
        vReal[i] = sq(vReal[i]) + sq(vImag[i]);
        vImag[i] = 0.0;
    }
}

static void sumEnergy(const float *bins, float *energy, int bin_size, int num_octaves, float scale)
{
    int bin = 1;
    for (int octave = 0; octave < num_octaves; octave++) {
        float sum = 0.0;
        for (int i = 0; i < bin_size; i++) {
            sum += real[bin++];
        }
        energy[octave] = scale * log(sum);
        bin_size *= 2;
    }
}

void setup(void)
{
    Serial.begin(115200);
    Serial.println("Configuring I2S...");
    esp_err_t err;

    // The I2S config as per the example
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),      // Receive, not transfer
        .sample_rate = 22627,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,   // although the SEL config should be left, it seems to transmit on right
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,       // Interrupt level 1
        .dma_buf_count = 8,     // number of buffers
        .dma_buf_len = BLOCK_SIZE       // samples per buffer
    };

    // The pin config as per the setup
    const i2s_pin_config_t pin_config = {
        .bck_io_num = 14,       // BCKL
        .ws_io_num = 15,        // LRCL
        .data_out_num = -1,     // not used (only for speakers)
        .data_in_num = 32       // DOUT
    };

    // Configuring the I2S driver and pins.
    // This function must be called before any I2S driver read/write operations.
    err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed installing driver: %d\n", err);
        while (true);
    }
    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Failed setting pin: %d\n", err);
        while (true);
    }
    Serial.println("I2S driver installed.");
}


void loop(void)
{
    static int32_t samples[BLOCK_SIZE];

    // Read multiple samples at once and calculate the sound pressure
    size_t num_bytes_read;
    esp_err_t err = i2s_read(I2S_PORT,
                             (char *) samples,
                             BLOCK_SIZE,        // the doc says bytes, but its elements.
                             &num_bytes_read,
                             portMAX_DELAY);    // no timeout
    int samples_read = num_bytes_read / 8;

    // integer to float
    integerToFloat(samples, real, imag, SAMPLES);

    // apply flat top window, optimal for energy calculations
    fft.Windowing(FFT_WIN_TYP_FLT_TOP, FFT_FORWARD);
    fft.Compute(FFT_FORWARD);

    // calculate energy in each bin
    calculateEnergy(real, imag, SAMPLES);

    // sum up energy in bin for each octave
    sumEnergy(real, energy, 1, OCTAVES, 1.0);

    // show energy
    for (int i = 0; i < OCTAVES; i++) {
        print(" %6.2f", energy[i]);
    }
    print("\n");

#if 0
    if (samples_read > 0) {
        float mean = 0;
        int32_t val;
        for (int i = 0; i < samples_read; ++i) {
            val = samples[i] / 256;
            if (val < 0) {
                val = -val;
            }
            mean += val;
        }
        for (float i = mean / samples_read; i >= 1024; i /= 1.5) {
            Serial.print("#");
        }
        Serial.println();
    }
#endif
}

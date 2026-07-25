#include "analog_input.h"
#include "ads1115.h"
#include "board_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h>

/* Schematic channel order: AN1<-AIN3, AN2<-AIN2, AN3<-AIN1, AN4<-AIN0. */
static const uint8_t s_ads_channel[BOARD_ANALOG_COUNT] = {3, 2, 1, 0};
static analog_mode_t s_mode[BOARD_ANALOG_COUNT];
static int16_t s_raw[BOARD_ANALOG_COUNT];
static float s_adc_v[BOARD_ANALOG_COUNT];
static SemaphoreHandle_t s_lock;

esp_err_t analog_input_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    for (int i = 0; i < BOARD_ANALOG_COUNT; ++i) {
        s_mode[i] = ANALOG_MODE_VOLTAGE_0_10V;
    }
    return ads1115_init();
}

void analog_input_sample_all(void)
{
    for (int i = 0; i < BOARD_ANALOG_COUNT; ++i) {
        int16_t raw;
        if (ads1115_read_raw(s_ads_channel[i], &raw) == ESP_OK) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_raw[i] = raw;
            s_adc_v[i] = (float)raw * 4.096f / 32768.0f;
            xSemaphoreGive(s_lock);
        }
    }
}

esp_err_t analog_input_set_mode(uint8_t channel, analog_mode_t mode)
{
    if (channel >= BOARD_ANALOG_COUNT || mode > ANALOG_MODE_CURRENT_4_20MA)
        return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_mode[channel] = mode;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

analog_mode_t analog_input_get_mode(uint8_t channel)
{
    if (channel >= BOARD_ANALOG_COUNT) return ANALOG_MODE_VOLTAGE_0_10V;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    analog_mode_t mode = s_mode[channel];
    xSemaphoreGive(s_lock);
    return mode;
}

int16_t analog_input_get_raw(uint8_t channel)
{
    if (channel >= BOARD_ANALOG_COUNT) return 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int16_t raw = s_raw[channel];
    xSemaphoreGive(s_lock);
    return raw;
}

uint16_t analog_input_get_adc_mv(uint8_t channel)
{
    if (channel >= BOARD_ANALOG_COUNT) return 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    float v = s_adc_v[channel];
    xSemaphoreGive(s_lock);
    if (v < 0) v = 0;
    return (uint16_t)lroundf(v * 1000.0f);
}

uint16_t analog_input_get_engineering_x100(uint8_t channel)
{
    if (channel >= BOARD_ANALOG_COUNT) return 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    float adc_v = s_adc_v[channel];
    analog_mode_t mode = s_mode[channel];
    xSemaphoreGive(s_lock);
    if (adc_v < 0) adc_v = 0;

    /* 27k/10k divider: ADC = input * 10/(27+10). */
    const float terminal_v = adc_v * 3.7f;
    float value = (mode == ANALOG_MODE_CURRENT_4_20MA)
                    ? terminal_v / 100.0f * 1000.0f  /* corrected 100 ohm shunt */
                    : terminal_v;
    if (value < 0) value = 0;
    if (value > 655.35f) value = 655.35f;
    return (uint16_t)lroundf(value * 100.0f);
}

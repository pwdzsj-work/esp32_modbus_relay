#pragma once
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    ANALOG_MODE_VOLTAGE_0_10V = 0,
    ANALOG_MODE_CURRENT_4_20MA = 1,
} analog_mode_t;

esp_err_t analog_input_init(void);
void analog_input_sample_all(void);
esp_err_t analog_input_set_mode(uint8_t channel, analog_mode_t mode);
analog_mode_t analog_input_get_mode(uint8_t channel);
int16_t analog_input_get_raw(uint8_t channel);
uint16_t analog_input_get_adc_mv(uint8_t channel);
uint16_t analog_input_get_engineering_x100(uint8_t channel);

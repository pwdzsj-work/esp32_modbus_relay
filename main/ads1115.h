#pragma once
#include <stdint.h>
#include "esp_err.h"

esp_err_t ads1115_init(void);
esp_err_t ads1115_read_raw(uint8_t channel, int16_t *raw);
esp_err_t ads1115_read_voltage(uint8_t channel, float *volts);

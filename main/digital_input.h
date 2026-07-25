#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t digital_input_init(void);
bool digital_input_get(uint8_t channel);
uint8_t digital_input_get_mask(void);
void digital_input_poll_and_log(void);

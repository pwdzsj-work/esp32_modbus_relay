#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t relay_driver_init(void);
esp_err_t relay_driver_set(uint8_t channel, bool on);
bool relay_driver_get_commanded(uint8_t channel);
uint8_t relay_driver_get_commanded_mask(void);
esp_err_t relay_driver_set_mask(uint8_t mask);

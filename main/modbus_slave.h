#pragma once
#include <stdint.h>
#include "esp_err.h"

esp_err_t modbus_slave_init(void);
uint8_t modbus_slave_get_address(void);

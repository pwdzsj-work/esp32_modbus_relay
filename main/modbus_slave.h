#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t modbus_slave_init(void);
uint8_t modbus_slave_get_address(void);
void modbus_slave_process_lora_frame(uint8_t *frame, size_t length);

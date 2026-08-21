#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define RELAY_SCHEDULE_MODBUS_REG_BASE       0x0400
#define RELAY_SCHEDULE_MODBUS_FIELDS         5
#define RELAY_SCHEDULE_MODBUS_REGISTER_COUNT 20

/* Configures SW3 for runtime detection. */
esp_err_t web_config_start_if_requested(void);

/*
 * Call periodically. Each three-second SW3 press toggles the Wi-Fi access
 * point and web server. SW3 must be released between toggle operations.
 */
void web_config_poll(void);

/*
 * Daily schedule holding registers, five registers per relay:
 * enabled, on hour, on minute, off hour, off minute.
 */
bool web_config_schedule_read_register(uint16_t address, uint16_t *value);
bool web_config_schedule_write_registers(uint16_t start,
                                         const uint16_t *values,
                                         size_t count);

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define RV3028_MODBUS_REG_YEAR    0x0300
#define RV3028_MODBUS_REG_MONTH   0x0301
#define RV3028_MODBUS_REG_DATE    0x0302
#define RV3028_MODBUS_REG_HOUR    0x0303
#define RV3028_MODBUS_REG_MINUTE  0x0304
#define RV3028_MODBUS_REG_SECOND  0x0305
#define RV3028_MODBUS_REG_COUNT   6

typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t date;
    uint8_t weekday;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} rv3028_datetime_t;

esp_err_t rv3028_init(void);
esp_err_t rv3028_get_time(rv3028_datetime_t *datetime, bool *valid);
esp_err_t rv3028_set_time(const rv3028_datetime_t *datetime);
void rv3028_poll_and_log(void);

bool rv3028_modbus_read_register(uint16_t address, uint16_t *value);
bool rv3028_modbus_write_register(uint16_t address, uint16_t value);
bool rv3028_modbus_write_registers(uint16_t start, const uint16_t *values,
                                   size_t count);

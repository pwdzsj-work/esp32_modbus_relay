#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/*
 * RS485 command registers (Modbus holding registers):
 *
 * 0x0200 R/W command argument
 * 0x0201 R/W command trigger (writing executes the command)
 * 0x0202 R   result of the last command
 * 0x0203 R   commanded relay mask
 *
 * Commands:
 * 1: Set one relay. Argument bits 7:0 = channel (1..4), bit 8 = state.
 * 2: Set all relays from argument bits 3:0.
 * 3: Turn all relays off. Argument is ignored.
 * 4: Turn all relays on. Argument is ignored.
 */
#define RS485_CTRL_REG_ARGUMENT       0x0200
#define RS485_CTRL_REG_COMMAND        0x0201
#define RS485_CTRL_REG_RESULT         0x0202
#define RS485_CTRL_REG_RELAY_MASK     0x0203

typedef enum {
    RS485_CTRL_CMD_SET_RELAY = 1,
    RS485_CTRL_CMD_SET_MASK = 2,
    RS485_CTRL_CMD_ALL_OFF = 3,
    RS485_CTRL_CMD_ALL_ON = 4,
} rs485_control_command_t;

typedef enum {
    RS485_CTRL_RESULT_OK = 0,
    RS485_CTRL_RESULT_INVALID_COMMAND = 1,
    RS485_CTRL_RESULT_INVALID_ARGUMENT = 2,
    RS485_CTRL_RESULT_DRIVER_ERROR = 3,
} rs485_control_result_t;

esp_err_t rs485_control_init(void);
bool rs485_control_read_register(uint16_t address, uint16_t *value);
bool rs485_control_write_register(uint16_t address, uint16_t value);

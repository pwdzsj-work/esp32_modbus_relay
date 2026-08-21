#include "rs485_control.h"
#include "board_config.h"
#include "relay_driver.h"

static uint16_t s_argument;
static uint16_t s_last_command;
static rs485_control_result_t s_result;

static void execute_command(uint16_t command)
{
    esp_err_t err = ESP_OK;

    switch (command) {
    case RS485_CTRL_CMD_SET_RELAY: {
        uint8_t channel = (uint8_t)(s_argument & 0xFF);
        bool on = (s_argument & 0x0100) != 0;
        if (channel == 0 || channel > BOARD_RELAY_COUNT) {
            s_result = RS485_CTRL_RESULT_INVALID_ARGUMENT;
            return;
        }
        err = relay_driver_set(channel - 1, on);
        break;
    }
    case RS485_CTRL_CMD_SET_MASK:
        if (s_argument & ~((1U << BOARD_RELAY_COUNT) - 1U)) {
            s_result = RS485_CTRL_RESULT_INVALID_ARGUMENT;
            return;
        }
        err = relay_driver_set_mask((uint8_t)s_argument);
        break;
    case RS485_CTRL_CMD_ALL_OFF:
        err = relay_driver_set_mask(0);
        break;
    case RS485_CTRL_CMD_ALL_ON:
        err = relay_driver_set_mask((1U << BOARD_RELAY_COUNT) - 1U);
        break;
    default:
        s_result = RS485_CTRL_RESULT_INVALID_COMMAND;
        return;
    }

    s_result = err == ESP_OK ? RS485_CTRL_RESULT_OK
                             : RS485_CTRL_RESULT_DRIVER_ERROR;
}

esp_err_t rs485_control_init(void)
{
    s_argument = 0;
    s_last_command = 0;
    s_result = RS485_CTRL_RESULT_OK;
    return ESP_OK;
}

bool rs485_control_read_register(uint16_t address, uint16_t *value)
{
    if (!value) return false;

    switch (address) {
    case RS485_CTRL_REG_ARGUMENT:
        *value = s_argument;
        return true;
    case RS485_CTRL_REG_COMMAND:
        *value = s_last_command;
        return true;
    case RS485_CTRL_REG_RESULT:
        *value = s_result;
        return true;
    case RS485_CTRL_REG_RELAY_MASK:
        *value = relay_driver_get_commanded_mask();
        return true;
    default:
        return false;
    }
}

bool rs485_control_write_register(uint16_t address, uint16_t value)
{
    if (address == RS485_CTRL_REG_ARGUMENT) {
        s_argument = value;
        return true;
    }
    if (address == RS485_CTRL_REG_COMMAND) {
        s_last_command = value;
        execute_command(value);
        return true;
    }
    return false;
}

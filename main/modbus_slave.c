#include "modbus_slave.h"
#include "board_config.h"
#include "relay_driver.h"
#include "digital_input.h"
#include "analog_input.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define RX_MAX 256
#define TX_MAX 256
#define REG_MODE_BASE 0x0100
#define REG_RELAY_MASK 0x0104
#define REG_INPUT_MASK 0x0105
#define REG_RAW_BASE 0x0000
#define REG_ADC_MV_BASE 0x0004
#define REG_VALUE_BASE 0x0008
#define REG_MODE_INPUT_BASE 0x000C

static const char *TAG = "MODBUS";
static TaskHandle_t s_task;

static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; ++i)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

static void put_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)value;
}

static uint16_t get_u16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static void send_frame(uint8_t *data, size_t len)
{
    uint16_t crc = crc16(data, len);
    data[len++] = (uint8_t)crc;
    data[len++] = (uint8_t)(crc >> 8);
    gpio_set_level(BOARD_RS485_DIR_GPIO, BOARD_RS485_DIR_TX_LEVEL);
    esp_rom_delay_us(50);
    uart_write_bytes(BOARD_RS485_UART, data, len);
    uart_wait_tx_done(BOARD_RS485_UART, pdMS_TO_TICKS(100));
    esp_rom_delay_us(100);
    gpio_set_level(BOARD_RS485_DIR_GPIO, BOARD_RS485_DIR_RX_LEVEL);
}

static void exception(uint8_t addr, uint8_t fc, uint8_t code)
{
    uint8_t tx[5] = {addr, (uint8_t)(fc | 0x80), code};
    send_frame(tx, 3);
}

static bool read_bit(bool coils, uint16_t address)
{
    if (address >= 4) return false;
    return coils ? relay_driver_get(address) : digital_input_get(address);
}

static bool read_input_reg(uint16_t address, uint16_t *value)
{
    if (address < 4) *value = (uint16_t)analog_input_get_raw(address);
    else if (address < 8) *value = analog_input_get_adc_mv(address - REG_ADC_MV_BASE);
    else if (address < 12) *value = analog_input_get_engineering_x100(address - REG_VALUE_BASE);
    else if (address < 16) *value = analog_input_get_mode(address - REG_MODE_INPUT_BASE);
    else if (address == 16) *value = relay_driver_get_mask();
    else if (address == 17) *value = digital_input_get_mask();
    else return false;
    return true;
}

static bool read_holding_reg(uint16_t address, uint16_t *value)
{
    if (address >= REG_MODE_BASE && address < REG_MODE_BASE + 4)
        *value = analog_input_get_mode(address - REG_MODE_BASE);
    else if (address == REG_RELAY_MASK) *value = relay_driver_get_mask();
    else if (address == REG_INPUT_MASK) *value = digital_input_get_mask();
    else return false;
    return true;
}

static bool write_holding_reg(uint16_t address, uint16_t value)
{
    if (address >= REG_MODE_BASE && address < REG_MODE_BASE + 4)
        return analog_input_set_mode(address - REG_MODE_BASE, (analog_mode_t)value) == ESP_OK;
    if (address == REG_RELAY_MASK)
        return relay_driver_set_mask((uint8_t)value) == ESP_OK;
    return false;
}

static void handle_request(uint8_t *rx, size_t len)
{
    if (len < 4 || rx[0] != BOARD_MODBUS_SLAVE_ADDR) return;
    uint16_t got_crc = rx[len - 2] | ((uint16_t)rx[len - 1] << 8);
    if (got_crc != crc16(rx, len - 2)) return;
    const uint8_t addr = rx[0], fc = rx[1];
    uint8_t tx[TX_MAX] = {addr, fc};

    if ((fc == 0x01 || fc == 0x02) && len == 8) {
        uint16_t start = get_u16(&rx[2]), count = get_u16(&rx[4]);
        if (!count || count > 4 || start + count > 4) {
            exception(addr, fc, 0x02); return;
        }
        tx[2] = (count + 7) / 8;
        tx[3] = 0;
        for (uint16_t i = 0; i < count; ++i)
            if (read_bit(fc == 0x01, start + i)) tx[3] |= 1U << i;
        send_frame(tx, 3 + tx[2]);
    } else if ((fc == 0x03 || fc == 0x04) && len == 8) {
        uint16_t start = get_u16(&rx[2]), count = get_u16(&rx[4]);
        if (!count || count > 32 || 3U + 2U * count > TX_MAX - 2) {
            exception(addr, fc, 0x03); return;
        }
        tx[2] = count * 2;
        for (uint16_t i = 0; i < count; ++i) {
            uint16_t value;
            bool ok = fc == 0x03 ? read_holding_reg(start + i, &value)
                                 : read_input_reg(start + i, &value);
            if (!ok) { exception(addr, fc, 0x02); return; }
            put_u16(&tx[3 + 2 * i], value);
        }
        send_frame(tx, 3 + 2 * count);
    } else if (fc == 0x05 && len == 8) {
        uint16_t coil = get_u16(&rx[2]), value = get_u16(&rx[4]);
        if (coil >= 4 || (value != 0x0000 && value != 0xFF00)) {
            exception(addr, fc, value == 0 && coil >= 4 ? 0x02 : 0x03); return;
        }
        relay_driver_set(coil, value == 0xFF00);
        send_frame(rx, 6);
    } else if (fc == 0x06 && len == 8) {
        uint16_t reg = get_u16(&rx[2]), value = get_u16(&rx[4]);
        if (!write_holding_reg(reg, value)) {
            exception(addr, fc, 0x03); return;
        }
        send_frame(rx, 6);
    } else if (fc == 0x0F && len >= 9) {
        uint16_t start = get_u16(&rx[2]), count = get_u16(&rx[4]);
        if (!count || start + count > 4 || rx[6] != (count + 7) / 8 ||
            len != (size_t)(9 + rx[6])) {
            exception(addr, fc, 0x03); return;
        }
        for (uint16_t i = 0; i < count; ++i)
            relay_driver_set(start + i, (rx[7 + i / 8] >> (i % 8)) & 1);
        memcpy(&tx[2], &rx[2], 4);
        send_frame(tx, 6);
    } else if (fc == 0x10 && len >= 9) {
        uint16_t start = get_u16(&rx[2]), count = get_u16(&rx[4]);
        if (!count || count > 16 || rx[6] != 2 * count ||
            len != (size_t)(9 + rx[6])) {
            exception(addr, fc, 0x03); return;
        }
        for (uint16_t i = 0; i < count; ++i) {
            if (!write_holding_reg(start + i, get_u16(&rx[7 + 2 * i]))) {
                exception(addr, fc, 0x03); return;
            }
        }
        memcpy(&tx[2], &rx[2], 4);
        send_frame(tx, 6);
    } else {
        exception(addr, fc, 0x01);
    }
}

static void modbus_task(void *arg)
{
    uint8_t rx[RX_MAX];
    while (true) {
        int len = uart_read_bytes(BOARD_RS485_UART, rx, sizeof(rx),
                                  pdMS_TO_TICKS(20));
        if (len > 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            int more = uart_read_bytes(BOARD_RS485_UART, rx + len,
                                       sizeof(rx) - len, 0);
            if (more > 0) len += more;
            handle_request(rx, len);
        }
    }
}

esp_err_t modbus_slave_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = BOARD_RS485_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(BOARD_RS485_UART, RX_MAX * 2, 0, 0, NULL, 0), TAG, "uart install");
    ESP_RETURN_ON_ERROR(uart_param_config(BOARD_RS485_UART, &uart_cfg), TAG, "uart config");
    ESP_RETURN_ON_ERROR(uart_set_pin(BOARD_RS485_UART, BOARD_RS485_TX_GPIO,
                                    BOARD_RS485_RX_GPIO, UART_PIN_NO_CHANGE,
                                    UART_PIN_NO_CHANGE), TAG, "uart pins");
    gpio_config_t dir_cfg = {
        .pin_bit_mask = 1ULL << BOARD_RS485_DIR_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&dir_cfg), TAG, "direction pin");
    gpio_set_level(BOARD_RS485_DIR_GPIO, BOARD_RS485_DIR_RX_LEVEL);
    if (xTaskCreate(modbus_task, "modbus_rtu", 4096, NULL, 10, &s_task) != pdPASS)
        return ESP_ERR_NO_MEM;
    return ESP_OK;
}

uint8_t modbus_slave_get_address(void)
{
    return BOARD_MODBUS_SLAVE_ADDR;
}

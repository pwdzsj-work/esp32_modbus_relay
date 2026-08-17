#include "modbus_slave.h"
#include "board_config.h"
#include "relay_driver.h"
#include "digital_input.h"
#include "analog_input.h"
#include "rv3028.h"
#include "rs485_control.h"
#include "lora_uart.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_log_buffer.h"
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

typedef enum {
    MODBUS_TRANSPORT_RS485,
    MODBUS_TRANSPORT_LORA,
} modbus_transport_t;

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

static const char *transport_name(modbus_transport_t transport)
{
    return transport == MODBUS_TRANSPORT_LORA ? "LoRa" : "RS485";
}

static void log_received_frame(const uint8_t *data, size_t len,
                               modbus_transport_t transport)
{
    if (len >= 4) {
        uint16_t received_crc = data[len - 2] |
                                ((uint16_t)data[len - 1] << 8);
        uint16_t calculated_crc = crc16(data, len - 2);
        ESP_LOGI(TAG, "%s RX: len=%u addr=%u function=0x%02X CRC=%s",
                 transport_name(transport), (unsigned)len, data[0], data[1],
                 received_crc == calculated_crc ? "OK" : "ERROR");
    } else {
        ESP_LOGW(TAG, "%s RX: short frame, len=%u",
                 transport_name(transport), (unsigned)len);
    }

    ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, len, ESP_LOG_INFO);
}

static void send_frame(uint8_t *data, size_t len,
                       modbus_transport_t transport)
{
    uint16_t crc = crc16(data, len);
    data[len++] = (uint8_t)crc;
    data[len++] = (uint8_t)(crc >> 8);

    if (transport == MODBUS_TRANSPORT_LORA) {
        esp_err_t err = lora_uart_send(data, len, pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "LoRa response failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "LoRa TX: len=%u", (unsigned)len);
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, len, ESP_LOG_INFO);
        }
        return;
    }

    gpio_set_level(BOARD_RS485_DIR_GPIO, BOARD_RS485_DIR_TX_LEVEL);
    esp_rom_delay_us(50);
    uart_write_bytes(BOARD_RS485_UART, data, len);
    uart_wait_tx_done(BOARD_RS485_UART, pdMS_TO_TICKS(100));
    esp_rom_delay_us(100);
    gpio_set_level(BOARD_RS485_DIR_GPIO, BOARD_RS485_DIR_RX_LEVEL);
}

static void exception(uint8_t addr, uint8_t fc, uint8_t code,
                      modbus_transport_t transport)
{
    uint8_t tx[5] = {addr, (uint8_t)(fc | 0x80), code};
    send_frame(tx, 3, transport);
}

static bool read_bit(bool coils, uint16_t address)
{
    if (coils) {
        if (address >= BOARD_RELAY_COUNT) return false;
        return relay_driver_get(address);
    }
    if (address >= BOARD_INPUT_COUNT) return false;
    return digital_input_get(address);
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
    else if (address == REG_RELAY_MASK)
        *value = relay_driver_get_mask();
    else if (address == REG_INPUT_MASK) *value = digital_input_get_mask();
    else if (rv3028_modbus_read_register(address, value)) return true;
    else return rs485_control_read_register(address, value);
    return true;
}

static bool write_holding_reg(uint16_t address, uint16_t value)
{
    if (address >= REG_MODE_BASE && address < REG_MODE_BASE + 4)
        return analog_input_set_mode(address - REG_MODE_BASE, (analog_mode_t)value) == ESP_OK;
    if (address == REG_RELAY_MASK)
        return relay_driver_set_mask((uint8_t)value) == ESP_OK;
    if (address >= RV3028_MODBUS_REG_YEAR &&
        address < RV3028_MODBUS_REG_YEAR + RV3028_MODBUS_REG_COUNT)
        return rv3028_modbus_write_register(address, value);
    return rs485_control_write_register(address, value);
}

static void handle_request(uint8_t *rx, size_t len,
                           modbus_transport_t transport)
{
    if (len < 4 || rx[0] != BOARD_MODBUS_SLAVE_ADDR) return;
    uint16_t got_crc = rx[len - 2] | ((uint16_t)rx[len - 1] << 8);
    if (got_crc != crc16(rx, len - 2)) return;
    const uint8_t addr = rx[0], fc = rx[1];
    uint8_t tx[TX_MAX] = {addr, fc};

    if (fc == 0x11 && len == 4) {
        /*
         * Report Server ID response:
         *   address, 0x11, byte count, server ID, run indicator, model...
         * The 0xFF run indicator means that the application is online.
         */
        static const char model[] = BOARD_DEVICE_MODEL;
        const size_t model_len = sizeof(model) - 1;
        tx[2] = (uint8_t)(2 + model_len);
        tx[3] = BOARD_MODBUS_SERVER_ID;
        tx[4] = 0xFF;
        memcpy(&tx[5], model, model_len);
        send_frame(tx, 5 + model_len, transport);
        ESP_LOGI(TAG, "Discovery reply sent via %s: slave=%u model=%s",
                 transport_name(transport), addr, model);
    } else if ((fc == 0x01 || fc == 0x02) && len == 8) {
        uint16_t start = get_u16(&rx[2]), count = get_u16(&rx[4]);
        uint16_t limit = fc == 0x01 ? BOARD_RELAY_COUNT : BOARD_INPUT_COUNT;
        if (!count || start >= limit || count > limit - start) {
            exception(addr, fc, 0x02, transport); return;
        }
        tx[2] = (count + 7) / 8;
        tx[3] = 0;
        for (uint16_t i = 0; i < count; ++i)
            if (read_bit(fc == 0x01, start + i)) tx[3] |= 1U << i;
        send_frame(tx, 3 + tx[2], transport);
    } else if ((fc == 0x03 || fc == 0x04) && len == 8) {
        uint16_t start = get_u16(&rx[2]), count = get_u16(&rx[4]);
        if (!count || count > 32 || 3U + 2U * count > TX_MAX - 2) {
            exception(addr, fc, 0x03, transport); return;
        }
        tx[2] = count * 2;
        for (uint16_t i = 0; i < count; ++i) {
            uint16_t value;
            bool ok = fc == 0x03 ? read_holding_reg(start + i, &value)
                                 : read_input_reg(start + i, &value);
            if (!ok) { exception(addr, fc, 0x02, transport); return; }
            put_u16(&tx[3 + 2 * i], value);
        }
        send_frame(tx, 3 + 2 * count, transport);
    } else if (fc == 0x05 && len == 8) {
        uint16_t coil = get_u16(&rx[2]), value = get_u16(&rx[4]);
        if (coil >= 4 || (value != 0x0000 && value != 0xFF00)) {
            exception(addr, fc, value == 0 && coil >= 4 ? 0x02 : 0x03,
                      transport); return;
        }
        relay_driver_set(coil, value == 0xFF00);
        send_frame(rx, 6, transport);
    } else if (fc == 0x06 && len == 8) {
        uint16_t reg = get_u16(&rx[2]), value = get_u16(&rx[4]);
        if (!write_holding_reg(reg, value)) {
            exception(addr, fc, 0x03, transport); return;
        }
        send_frame(rx, 6, transport);
    } else if (fc == 0x0F && len >= 9) {
        uint16_t start = get_u16(&rx[2]), count = get_u16(&rx[4]);
        if (!count || start + count > 4 || rx[6] != (count + 7) / 8 ||
            len != (size_t)(9 + rx[6])) {
            exception(addr, fc, 0x03, transport); return;
        }
        for (uint16_t i = 0; i < count; ++i)
            relay_driver_set(start + i, (rx[7 + i / 8] >> (i % 8)) & 1);
        memcpy(&tx[2], &rx[2], 4);
        send_frame(tx, 6, transport);
    } else if (fc == 0x10 && len >= 9) {
        uint16_t start = get_u16(&rx[2]), count = get_u16(&rx[4]);
        if (!count || count > 16 || rx[6] != 2 * count ||
            len != (size_t)(9 + rx[6])) {
            exception(addr, fc, 0x03, transport); return;
        }
        if (start >= RV3028_MODBUS_REG_YEAR &&
            start + count <=
                RV3028_MODBUS_REG_YEAR + RV3028_MODBUS_REG_COUNT) {
            uint16_t values[RV3028_MODBUS_REG_COUNT];
            for (uint16_t i = 0; i < count; ++i)
                values[i] = get_u16(&rx[7 + 2 * i]);
            if (!rv3028_modbus_write_registers(start, values, count)) {
                exception(addr, fc, 0x03, transport); return;
            }
        } else {
            for (uint16_t i = 0; i < count; ++i) {
                if (!write_holding_reg(start + i,
                                       get_u16(&rx[7 + 2 * i]))) {
                    exception(addr, fc, 0x03, transport); return;
                }
            }
        }
        memcpy(&tx[2], &rx[2], 4);
        send_frame(tx, 6, transport);
    } else {
        exception(addr, fc, 0x01, transport);
    }
}

void modbus_slave_process_lora_frame(uint8_t *frame, size_t length)
{
    if (!frame || length == 0) {
        return;
    }
    log_received_frame(frame, length, MODBUS_TRANSPORT_LORA);
    handle_request(frame, length, MODBUS_TRANSPORT_LORA);
}

static void modbus_task(void *arg)
{
    uint8_t rx[RX_MAX];
    TickType_t last_status_log = xTaskGetTickCount();
    ESP_LOGI(TAG, "RS485 receiver ready, waiting for Modbus request");

    while (true) {
        int len = uart_read_bytes(BOARD_RS485_UART, rx, sizeof(rx),
                                  pdMS_TO_TICKS(20));
        if (len > 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            int more = uart_read_bytes(BOARD_RS485_UART, rx + len,
                                       sizeof(rx) - len, 0);
            if (more > 0) len += more;
            log_received_frame(rx, len, MODBUS_TRANSPORT_RS485);
            handle_request(rx, len, MODBUS_TRANSPORT_RS485);
            last_status_log = xTaskGetTickCount();
        } else {
            TickType_t now = xTaskGetTickCount();
            if (now - last_status_log >= pdMS_TO_TICKS(5000)) {
                size_t buffered = 0;
                uart_get_buffered_data_len(BOARD_RS485_UART, &buffered);
                ESP_LOGI(TAG,
                         "RS485 waiting: 485C=%d RXD2=%d buffered=%u",
                         gpio_get_level(BOARD_RS485_DIR_GPIO),
                         gpio_get_level(BOARD_RS485_RX_GPIO),
                         (unsigned)buffered);
                last_status_log = now;
            }
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
    ESP_LOGI(TAG,
             "UART%d initialized: TX=GPIO%d RX=GPIO%d, 9600 8N1",
             BOARD_RS485_UART, BOARD_RS485_TX_GPIO, BOARD_RS485_RX_GPIO);
    ESP_LOGI(TAG,
             "485C direction control: GPIO%d, RX level=%d, TX level=%d",
             BOARD_RS485_DIR_GPIO, BOARD_RS485_DIR_RX_LEVEL,
             BOARD_RS485_DIR_TX_LEVEL);

    if (xTaskCreate(modbus_task, "modbus_rtu", 4096, NULL, 10, &s_task) != pdPASS)
        return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "Modbus receive task created successfully");
    ESP_LOGI(TAG,
             "LoRa discovery enabled: poll slave addresses with function 0x11");
    return ESP_OK;
}

uint8_t modbus_slave_get_address(void)
{
    return BOARD_MODBUS_SLAVE_ADDR;
}

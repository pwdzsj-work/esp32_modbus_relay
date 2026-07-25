#include "ads1115.h"
#include "board_config.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ADS_REG_CONVERSION 0x00
#define ADS_REG_CONFIG     0x01
#define ADS_OS_SINGLE      (1U << 15)
#define ADS_MUX_SINGLE(ch) ((uint16_t)(4U + (ch)) << 12)
#define ADS_PGA_4V096      (1U << 9)
#define ADS_MODE_SINGLE    (1U << 8)
#define ADS_DR_128SPS      (4U << 5)
#define ADS_COMP_DISABLE   3U

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

static esp_err_t write_reg(uint8_t reg, uint16_t value)
{
    uint8_t data[3] = { reg, (uint8_t)(value >> 8), (uint8_t)value };
    return i2c_master_transmit(s_dev, data, sizeof(data), 100);
}

static esp_err_t read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, data, 2, 100);
    if (err == ESP_OK) *value = ((uint16_t)data[0] << 8) | data[1];
    return err;
}

esp_err_t ads1115_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = BOARD_I2C_PORT,
        .sda_io_num = BOARD_I2C_SDA_GPIO,
        .scl_io_num = BOARD_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) return err;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BOARD_ADS1115_ADDR,
        .scl_speed_hz = BOARD_I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
}

esp_err_t ads1115_read_raw(uint8_t channel, int16_t *raw)
{
    if (!raw || channel > 3) return ESP_ERR_INVALID_ARG;
    uint16_t cfg = ADS_OS_SINGLE | ADS_MUX_SINGLE(channel) | ADS_PGA_4V096 |
                   ADS_MODE_SINGLE | ADS_DR_128SPS | ADS_COMP_DISABLE;
    esp_err_t err = write_reg(ADS_REG_CONFIG, cfg);
    if (err != ESP_OK) return err;

    for (int i = 0; i < 20; ++i) {
        vTaskDelay(pdMS_TO_TICKS(1));
        uint16_t status;
        err = read_reg(ADS_REG_CONFIG, &status);
        if (err != ESP_OK) return err;
        if (status & ADS_OS_SINGLE) {
            uint16_t value;
            err = read_reg(ADS_REG_CONVERSION, &value);
            if (err == ESP_OK) *raw = (int16_t)value;
            return err;
        }
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t ads1115_read_voltage(uint8_t channel, float *volts)
{
    if (!volts) return ESP_ERR_INVALID_ARG;
    int16_t raw;
    esp_err_t err = ads1115_read_raw(channel, &raw);
    if (err == ESP_OK) *volts = (float)raw * 4.096f / 32768.0f;
    return err;
}

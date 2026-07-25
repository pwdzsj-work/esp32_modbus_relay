#include "relay_driver.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const gpio_num_t s_gpio[BOARD_RELAY_COUNT] = {
    BOARD_RELAY1_GPIO, BOARD_RELAY2_GPIO, BOARD_RELAY3_GPIO, BOARD_RELAY4_GPIO
};
static uint8_t s_mask;
static SemaphoreHandle_t s_lock;

esp_err_t relay_driver_init(void)
{
    /* Set safe inactive levels before changing pins to output mode. */
    for (int i = 0; i < BOARD_RELAY_COUNT; ++i) {
        gpio_set_level(s_gpio[i], BOARD_RELAY_INACTIVE_LEVEL);
    }
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BOARD_RELAY1_GPIO) | (1ULL << BOARD_RELAY2_GPIO) |
                        (1ULL << BOARD_RELAY3_GPIO) | (1ULL << BOARD_RELAY4_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) return ESP_ERR_NO_MEM;
    s_mask = 0;
    return gpio_config(&cfg);
}

esp_err_t relay_driver_set(uint8_t channel, bool on)
{
    if (channel >= BOARD_RELAY_COUNT) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    esp_err_t err = gpio_set_level(s_gpio[channel],
                                   on ? BOARD_RELAY_ACTIVE_LEVEL : BOARD_RELAY_INACTIVE_LEVEL);
    if (err == ESP_OK) {
        if (on) s_mask |= (1U << channel);
        else s_mask &= ~(1U << channel);
    }
    xSemaphoreGive(s_lock);
    return err;
}

bool relay_driver_get(uint8_t channel)
{
    if (channel >= BOARD_RELAY_COUNT) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool on = (s_mask & (1U << channel)) != 0;
    xSemaphoreGive(s_lock);
    return on;
}

uint8_t relay_driver_get_mask(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    uint8_t mask = s_mask;
    xSemaphoreGive(s_lock);
    return mask;
}

esp_err_t relay_driver_set_mask(uint8_t mask)
{
    for (uint8_t i = 0; i < BOARD_RELAY_COUNT; ++i) {
        esp_err_t err = relay_driver_set(i, (mask & (1U << i)) != 0);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

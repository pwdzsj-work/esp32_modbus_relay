#include "digital_input.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "INPUT";
#define INPUT_SIGNAL_LOG_PERIOD_MS 1000

static const gpio_num_t s_gpio[BOARD_INPUT_COUNT] = {
    BOARD_NQ1_GPIO, BOARD_NQ2_GPIO, BOARD_NQ3_GPIO, BOARD_NQ4_GPIO
};
static uint8_t s_last_mask;
static TickType_t s_last_signal_log;
static bool s_initialized;

static void log_input_signals(uint8_t active_mask)
{
    int nq1_level = gpio_get_level(BOARD_NQ1_GPIO);
    int nq2_level = gpio_get_level(BOARD_NQ2_GPIO);
    int nq3_level = gpio_get_level(BOARD_NQ3_GPIO);
    int nq4_level = gpio_get_level(BOARD_NQ4_GPIO);

    ESP_LOGI(TAG,
             "GPIO levels: NQ1=%s NQ2=%s NQ3=%s NQ4=%s "
             "(active-low mask=0x%X)",
             nq1_level ? "HIGH" : "LOW",
             nq2_level ? "HIGH" : "LOW",
             nq3_level ? "HIGH" : "LOW",
             nq4_level ? "HIGH" : "LOW",
             active_mask & 0x0FU);
}

esp_err_t digital_input_init(void)
{
    gpio_config_t signal_cfg = {
        .pin_bit_mask = BOARD_NQ_SIGNAL_GPIO_MASK,
        .mode = GPIO_MODE_INPUT,
        /*
         * Keep NQ1..NQ4 at a defined high level when the external
         * optocoupler output is open.
         */
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&signal_cfg);
    if (err != ESP_OK) return err;

    s_last_mask = digital_input_get_mask();
    s_last_signal_log = xTaskGetTickCount();
    s_initialized = true;
    ESP_LOGI(TAG,
             "Signal inputs configured: NQ1=GPIO%d NQ2=GPIO%d "
             "NQ3=GPIO%d NQ4=GPIO%d",
             BOARD_NQ1_GPIO, BOARD_NQ2_GPIO,
             BOARD_NQ3_GPIO, BOARD_NQ4_GPIO);
    ESP_LOGI(TAG, "NQ1..NQ4 input mask=0x%02X", s_last_mask);
    log_input_signals(s_last_mask);
    return ESP_OK;
}

bool digital_input_get(uint8_t channel)
{
    if (channel >= BOARD_INPUT_COUNT) return false;
    return gpio_get_level(s_gpio[channel]) == BOARD_NQ_ACTIVE_LEVEL;
}

uint8_t digital_input_get_mask(void)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < BOARD_INPUT_COUNT; ++i) {
        if (digital_input_get(i)) mask |= (1U << i);
    }
    return mask;
}

void digital_input_poll_and_log(void)
{
    if (!s_initialized) return;

    uint8_t mask = digital_input_get_mask();
    uint8_t changed = mask ^ s_last_mask;

    if (changed) {
        for (uint8_t i = 0; i < BOARD_INPUT_COUNT; ++i) {
            if (changed & (1U << i)) {
                ESP_LOGI(TAG, "NQ%u -> %u (GPIO%d=%d)", i + 1,
                         (mask >> i) & 1U, s_gpio[i],
                         gpio_get_level(s_gpio[i]));
            }
        }
        ESP_LOGI(TAG, "NQ1..NQ4 mask changed: 0x%02X", mask);
        s_last_mask = mask;
    }

    TickType_t now = xTaskGetTickCount();
    bool signal_changed = (changed & 0x0FU) != 0;
    if (signal_changed ||
        now - s_last_signal_log >=
            pdMS_TO_TICKS(INPUT_SIGNAL_LOG_PERIOD_MS)) {
        log_input_signals(mask);
        s_last_signal_log = now;
    }
}

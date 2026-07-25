#include "digital_input.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "INPUT";
#define RELAY_FEEDBACK_FIRST_BIT 4
#define RELAY_FEEDBACK_LOG_PERIOD_MS 1000

static const gpio_num_t s_gpio[BOARD_INPUT_COUNT] = {
    BOARD_NQ1_GPIO, BOARD_NQ2_GPIO, BOARD_NQ3_GPIO, BOARD_NQ4_GPIO,
    BOARD_NQ8_GPIO, BOARD_NQ7_GPIO, BOARD_NQ6_GPIO, BOARD_NQ5_GPIO
};
static uint8_t s_last_mask;
static TickType_t s_last_feedback_log;
static bool s_initialized;

static void log_relay_feedback(uint8_t input_mask)
{
    uint8_t feedback = input_mask >> RELAY_FEEDBACK_FIRST_BIT;
    ESP_LOGI(TAG,
             "Relay feedback: R1=%s R2=%s R3=%s R4=%s (mask=0x%X)",
             (feedback & (1U << 0)) ? "PULLED_IN" : "RELEASED",
             (feedback & (1U << 1)) ? "PULLED_IN" : "RELEASED",
             (feedback & (1U << 2)) ? "PULLED_IN" : "RELEASED",
             (feedback & (1U << 3)) ? "PULLED_IN" : "RELEASED",
             feedback & 0x0FU);
}

esp_err_t digital_input_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BOARD_NQ1_GPIO) | (1ULL << BOARD_NQ2_GPIO) |
                        (1ULL << BOARD_NQ3_GPIO) | (1ULL << BOARD_NQ4_GPIO) |
                        (1ULL << BOARD_NQ5_GPIO) | (1ULL << BOARD_NQ6_GPIO) |
                        (1ULL << BOARD_NQ7_GPIO) | (1ULL << BOARD_NQ8_GPIO),
        .mode = GPIO_MODE_INPUT,
        /* External PC817 collector pull-ups already exist on the schematic. */
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;

    s_last_mask = digital_input_get_mask();
    s_last_feedback_log = xTaskGetTickCount();
    s_initialized = true;
    ESP_LOGI(TAG, "NQ1..NQ8 initialized, mask=0x%02X", s_last_mask);
    log_relay_feedback(s_last_mask);
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
                ESP_LOGI(TAG, "NQ%u -> %u (GPIO%d=%d)%s", i + 1,
                         (mask >> i) & 1U, s_gpio[i],
                         gpio_get_level(s_gpio[i]),
                         i >= RELAY_FEEDBACK_FIRST_BIT
                             ? " [relay feedback]" : "");
            }
        }
        ESP_LOGI(TAG, "NQ1..NQ8 mask changed: 0x%02X", mask);
        s_last_mask = mask;
    }

    TickType_t now = xTaskGetTickCount();
    bool feedback_changed =
        (changed & (0x0FU << RELAY_FEEDBACK_FIRST_BIT)) != 0;
    if (feedback_changed ||
        now - s_last_feedback_log >=
            pdMS_TO_TICKS(RELAY_FEEDBACK_LOG_PERIOD_MS)) {
        log_relay_feedback(mask);
        s_last_feedback_log = now;
    }
}

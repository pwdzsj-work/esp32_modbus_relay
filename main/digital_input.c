#include "digital_input.h"
#include "board_config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "INPUT";
#define RELAY_FEEDBACK_LOG_PERIOD_MS 1000
#define INPUT_SIGNAL_LOG_PERIOD_MS 1000

static const gpio_num_t s_gpio[BOARD_INPUT_COUNT] = {
    BOARD_NQ1_GPIO, BOARD_NQ2_GPIO, BOARD_NQ3_GPIO, BOARD_NQ4_GPIO,
    BOARD_NQ5_GPIO, BOARD_NQ6_GPIO, BOARD_NQ7_GPIO, BOARD_NQ8_GPIO
};
static uint8_t s_last_mask;
static TickType_t s_last_feedback_log;
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

static void log_relay_feedback(uint8_t input_mask)
{
    uint8_t feedback = input_mask >> BOARD_RELAY_FEEDBACK_FIRST_INPUT;
    ESP_LOGI(TAG,
             "Relay drive feedback: R1=%s R2=%s R3=%s R4=%s (mask=0x%X)",
             (feedback & (1U << 0)) ? "ACTIVE" : "INACTIVE",
             (feedback & (1U << 1)) ? "ACTIVE" : "INACTIVE",
             (feedback & (1U << 2)) ? "ACTIVE" : "INACTIVE",
             (feedback & (1U << 3)) ? "ACTIVE" : "INACTIVE",
             feedback & 0x0FU);
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

    gpio_config_t feedback_cfg = signal_cfg;
    feedback_cfg.pin_bit_mask = BOARD_NQ_FEEDBACK_GPIO_MASK;
    /* GPIO34..GPIO39 do not provide ESP32 internal pull resistors. */
    feedback_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    err = gpio_config(&feedback_cfg);
    if (err != ESP_OK) return err;

    s_last_mask = digital_input_get_mask();
    s_last_feedback_log = xTaskGetTickCount();
    s_last_signal_log = s_last_feedback_log;
    s_initialized = true;
    ESP_LOGI(TAG,
             "Signal inputs configured: NQ1=GPIO%d NQ2=GPIO%d "
             "NQ3=GPIO%d NQ4=GPIO%d",
             BOARD_NQ1_GPIO, BOARD_NQ2_GPIO,
             BOARD_NQ3_GPIO, BOARD_NQ4_GPIO);
    ESP_LOGI(TAG, "NQ1..NQ8 input mask=0x%02X", s_last_mask);
    log_input_signals(s_last_mask);
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
                         i >= BOARD_RELAY_FEEDBACK_FIRST_INPUT
                             ? " [relay feedback]" : "");
            }
        }
        ESP_LOGI(TAG, "NQ1..NQ8 mask changed: 0x%02X", mask);
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

    bool feedback_changed =
        (changed & (0x0FU << BOARD_RELAY_FEEDBACK_FIRST_INPUT)) != 0;
    if (feedback_changed ||
        now - s_last_feedback_log >=
            pdMS_TO_TICKS(RELAY_FEEDBACK_LOG_PERIOD_MS)) {
        log_relay_feedback(mask);
        s_last_feedback_log = now;
    }
}

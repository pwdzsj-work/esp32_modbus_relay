#include "digital_input.h"
#include "board_config.h"
#include "driver/gpio.h"

static const gpio_num_t s_gpio[BOARD_INPUT_COUNT] = {
    BOARD_NQ1_GPIO, BOARD_NQ2_GPIO, BOARD_NQ3_GPIO, BOARD_NQ4_GPIO
};

esp_err_t digital_input_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << BOARD_NQ1_GPIO) | (1ULL << BOARD_NQ2_GPIO) |
                        (1ULL << BOARD_NQ3_GPIO) | (1ULL << BOARD_NQ4_GPIO),
        .mode = GPIO_MODE_INPUT,
        /* External PC817 collector pull-ups already exist on the schematic. */
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&cfg);
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

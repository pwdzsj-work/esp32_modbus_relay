#include "lora_periodic_sender.h"

#include "lora_uart.h"

#include "esp_log.h"
#include "esp_log_buffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LORA_TEST_SEND_PERIOD_MS  2000
#define LORA_TEST_SEND_TIMEOUT_MS 1000

static const char *TAG = "LORA_TX";
static TaskHandle_t s_sender_task;

/* Transparent test command sent automatically after power-up. */
static const uint8_t s_test_command[] = "LORA_TEST\r\n";

static void sender_task(void *argument)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    ESP_LOGI(TAG, "Periodic sender started: interval=%d ms",
             LORA_TEST_SEND_PERIOD_MS);

    while (true) {
        esp_err_t err = lora_uart_send(s_test_command,
                                       sizeof(s_test_command) - 1,
                                       pdMS_TO_TICKS(
                                           LORA_TEST_SEND_TIMEOUT_MS));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Sent %u test bytes",
                     (unsigned)(sizeof(s_test_command) - 1));
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, s_test_command,
                                     sizeof(s_test_command) - 1,
                                     ESP_LOG_INFO);
        } else {
            ESP_LOGE(TAG, "Periodic send failed: %s",
                     esp_err_to_name(err));
        }

        xTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(LORA_TEST_SEND_PERIOD_MS));
    }
}

esp_err_t lora_periodic_sender_start(void)
{
    if (!lora_uart_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_sender_task) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xTaskCreate(sender_task, "lora_test_tx", 3072, NULL, 8,
                    &s_sender_task) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

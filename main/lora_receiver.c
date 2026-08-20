#include "lora_receiver.h"

#include "lora_discovery.h"
#include "lora_uart.h"
#include "modbus_slave.h"

#include "esp_log.h"
#include "esp_log_buffer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LORA_RX_FRAME_MAX       256
#define LORA_RX_WAIT_MS         100
#define LORA_RX_FRAME_GAP_MS    5

static const char *TAG = "LORA_RX";
static TaskHandle_t s_receive_task;

static void receive_task(void *argument)
{
    uint8_t data[LORA_RX_FRAME_MAX];

    ESP_LOGI(TAG, "Receive task started");

    while (true) {
        int length = lora_uart_receive(data, sizeof(data),
                                       pdMS_TO_TICKS(LORA_RX_WAIT_MS));
        if (length <= 0) {
            continue;
        }

        /*
         * Treat a short UART idle period as the end of one transparent LoRa
         * packet. This also collects bytes that arrived in separate UART FIFO
         * chunks.
         */
        while (length < sizeof(data)) {
            int more = lora_uart_receive(data + length,
                                         sizeof(data) - length,
                                         pdMS_TO_TICKS(LORA_RX_FRAME_GAP_MS));
            if (more <= 0) {
                break;
            }
            length += more;
        }

        ESP_LOGI(TAG, "Received %d byte%s",
                 length, length == 1 ? "" : "s");
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, length, ESP_LOG_INFO);
        if (!lora_discovery_process_frame(data, (size_t)length)) {
            modbus_slave_process_lora_frame(data, (size_t)length);
        }

        if (length == sizeof(data)) {
            ESP_LOGW(TAG,
                     "Receive buffer full; remaining bytes will be logged separately");
        }
    }
}

esp_err_t lora_receiver_init(void)
{
    if (s_receive_task) {
        return ESP_ERR_INVALID_STATE;
    }

    lora_uart_config_t config = lora_uart_get_default_config();
    esp_err_t err = lora_uart_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LoRa UART initialization failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = lora_discovery_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LoRa discovery initialization failed: %s",
                 esp_err_to_name(err));
        lora_uart_deinit();
        return err;
    }

    if (xTaskCreate(receive_task, "lora_rx", 3072, NULL, 9,
                    &s_receive_task) != pdPASS) {
        lora_uart_deinit();
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "LoRa receive processing initialized");
    return ESP_OK;
}

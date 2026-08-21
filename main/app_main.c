#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "relay_driver.h"
#include "digital_input.h"
#include "analog_input.h"
#include "rv3028.h"
#include "modbus_slave.h"
#include "rs485_control.h"
#include "lora_receiver.h"
#include "web_config.h"

static const char *TAG = "APP";
<<<<<<< HEAD
#define APP_BUILD_TAG "rs485-lora-mac-discovery-20260820"
=======
#define APP_BUILD_TAG "lora-relay-schedule-sync-20260821"
>>>>>>> 5008f76 (适配终端和上位机通信协议机制)

void app_main(void)
{
    ESP_LOGI(TAG, "Application starting");
    ESP_LOGI(TAG, "Firmware build: %s", APP_BUILD_TAG);

    ESP_ERROR_CHECK(relay_driver_init());
    ESP_LOGI(TAG, "Relay driver initialized");

    ESP_ERROR_CHECK(digital_input_init());
    ESP_LOGI(TAG, "Digital inputs initialized");

    ESP_ERROR_CHECK(analog_input_init());
    ESP_LOGI(TAG, "ADS1115 analog inputs initialized");

    ESP_ERROR_CHECK(rv3028_init());
    ESP_LOGI(TAG, "RV-3028 real-time clock initialized");

    ESP_ERROR_CHECK(rs485_control_init());
    ESP_LOGI(TAG, "RS485 command control initialized");

    /* NVS-backed schedules must be ready before either Modbus transport can
     * accept schedule reads or writes. */
    ESP_ERROR_CHECK(web_config_start_if_requested());
    ESP_ERROR_CHECK(modbus_slave_init());
    ESP_ERROR_CHECK(lora_receiver_init());

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
        ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        ESP_ERROR_CHECK(esp_ota_mark_app_valid_cancel_rollback());
        ESP_LOGI(TAG, "OTA image '%s' marked valid", running->label);
    }

    ESP_LOGI(TAG,
             "Initialization complete: RS485/LoRa Modbus RTU slave=%u, 9600 8N1",
             modbus_slave_get_address());

    while (true) {
        analog_input_sample_all();
        digital_input_poll_and_log();
        rv3028_poll_and_log();
        web_config_poll();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

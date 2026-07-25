#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "relay_driver.h"
#include "digital_input.h"
#include "analog_input.h"
#include "modbus_slave.h"

static const char *TAG = "APP";

void app_main(void)
{
    ESP_ERROR_CHECK(relay_driver_init());
    ESP_ERROR_CHECK(digital_input_init());
    ESP_ERROR_CHECK(analog_input_init());
    ESP_ERROR_CHECK(modbus_slave_init());

    ESP_LOGI(TAG, "Modbus RTU relay controller started, slave=%u, 9600 8N1",
             modbus_slave_get_address());

    while (true) {
        analog_input_sample_all();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

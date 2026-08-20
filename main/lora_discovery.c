#include "lora_discovery.h"

#include "board_config.h"
#include "lora_uart.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <string.h>

#define DISCOVERY_HEADER_0             0xAA
#define DISCOVERY_HEADER_1             0x55
#define DISCOVERY_VERSION              0x01
#define DISCOVERY_COMMAND_REQUEST      0x01
#define DISCOVERY_COMMAND_RESPONSE     0x81
#define DISCOVERY_REQUEST_PAYLOAD_LEN  5
#define DISCOVERY_REQUEST_FRAME_LEN    12
#define DISCOVERY_RESPONSE_FIXED_LEN   17
#define DISCOVERY_RESPONSE_FRAME_MAX   256

#define CAPABILITY_RELAY_CONTROL       (1UL << 0)
#define CAPABILITY_DIGITAL_INPUT       (1UL << 1)
#define CAPABILITY_ANALOG_INPUT        (1UL << 2)
#define CAPABILITY_RTC_TIME            (1UL << 3)
#define CAPABILITY_DAILY_SCHEDULE      (1UL << 4)
#define CAPABILITY_VOLTAGE             (1UL << 5)
#define CAPABILITY_CURRENT             (1UL << 6)
#define BOARD_CAPABILITIES             (CAPABILITY_RELAY_CONTROL | \
                                        CAPABILITY_DIGITAL_INPUT | \
                                        CAPABILITY_ANALOG_INPUT | \
                                        CAPABILITY_RTC_TIME | \
                                        CAPABILITY_DAILY_SCHEDULE | \
                                        CAPABILITY_VOLTAGE | \
                                        CAPABILITY_CURRENT)

typedef struct {
    uint16_t nonce;
    uint8_t slot_count;
    uint16_t slot_duration_ms;
    uint32_t generation;
} discovery_request_t;

static const char *TAG = "LORA_DISCOVERY";
static uint8_t s_mac[6];
static SemaphoreHandle_t s_lock;
static TaskHandle_t s_worker_task;
static discovery_request_t s_pending_request;
static uint16_t s_last_nonce;
static bool s_last_nonce_valid;

static uint16_t crc16_modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    while (length--) {
        crc ^= *data++;
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 1U) ? (crc >> 1) ^ 0xA001U : crc >> 1;
        }
    }
    return crc;
}

static uint16_t get_u16_be(const uint8_t *data)
{
    return ((uint16_t)data[0] << 8) | data[1];
}

static void put_u16_be(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
}

static void put_u32_be(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24);
    data[1] = (uint8_t)(value >> 16);
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)value;
}

static esp_err_t send_discovery_response(const discovery_request_t *request)
{
    const size_t sku_length = strlen(BOARD_PRODUCT_SKU);
    const size_t payload_length = DISCOVERY_RESPONSE_FIXED_LEN + sku_length;
    const size_t bytes_before_crc = 5 + payload_length;
    if (payload_length > UINT8_MAX ||
        bytes_before_crc + 2 > DISCOVERY_RESPONSE_FRAME_MAX) {
        ESP_LOGE(TAG, "SKU is too long for discovery response");
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t frame[DISCOVERY_RESPONSE_FRAME_MAX];
    frame[0] = DISCOVERY_HEADER_0;
    frame[1] = DISCOVERY_HEADER_1;
    frame[2] = DISCOVERY_VERSION;
    frame[3] = DISCOVERY_COMMAND_RESPONSE;
    frame[4] = (uint8_t)payload_length;
    put_u16_be(&frame[5], request->nonce);
    memcpy(&frame[7], s_mac, sizeof(s_mac));
    frame[13] = BOARD_MODBUS_SLAVE_ADDR;
    put_u32_be(&frame[14], BOARD_CAPABILITIES);
    frame[18] = BOARD_RELAY_COUNT;
    frame[19] = BOARD_INPUT_COUNT;
    frame[20] = BOARD_ANALOG_COUNT;
    frame[21] = (uint8_t)sku_length;
    memcpy(&frame[22], BOARD_PRODUCT_SKU, sku_length);

    const uint16_t crc = crc16_modbus(frame, bytes_before_crc);
    frame[bytes_before_crc] = (uint8_t)crc;
    frame[bytes_before_crc + 1] = (uint8_t)(crc >> 8);

    esp_err_t err = lora_uart_send(frame, bytes_before_crc + 2,
                                   pdMS_TO_TICKS(1000));
    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "Discovery response sent: nonce=0x%04X addr=%u SKU=%s",
                 request->nonce, BOARD_MODBUS_SLAVE_ADDR,
                 BOARD_PRODUCT_SKU);
    } else {
        ESP_LOGE(TAG, "Discovery response failed: %s",
                 esp_err_to_name(err));
    }
    return err;
}

static void discovery_worker(void *argument)
{
    (void)argument;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (true) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            discovery_request_t request = s_pending_request;
            xSemaphoreGive(s_lock);

            uint8_t slot_data[8];
            memcpy(slot_data, s_mac, sizeof(s_mac));
            put_u16_be(&slot_data[6], request.nonce);
            const uint16_t slot_crc = crc16_modbus(slot_data,
                                                   sizeof(slot_data));
            const uint8_t slot = (uint8_t)(slot_crc % request.slot_count);
            const uint32_t delay_ms =
                (uint32_t)slot * request.slot_duration_ms;

            ESP_LOGI(TAG,
                     "nonce=0x%04X CRC=0x%04X slot=%u/%u delay=%lu ms",
                     request.nonce, slot_crc, slot, request.slot_count,
                     (unsigned long)delay_ms);

            TickType_t delay_ticks = pdMS_TO_TICKS(delay_ms);
            if (delay_ms > 0 && delay_ticks == 0) {
                delay_ticks = 1;
            }
            if (ulTaskNotifyTake(pdTRUE, delay_ticks) > 0) {
                ESP_LOGI(TAG, "Pending response replaced by a newer nonce");
                continue;
            }

            xSemaphoreTake(s_lock, portMAX_DELAY);
            if (request.generation != s_pending_request.generation) {
                xSemaphoreGive(s_lock);
                continue;
            }
            send_discovery_response(&request);
            xSemaphoreGive(s_lock);
            break;
        }
    }
}

esp_err_t lora_discovery_init(void)
{
    if (s_worker_task) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = esp_efuse_mac_get_default(s_mac);
    if (err != ESP_OK) {
        return err;
    }

    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) {
        return ESP_ERR_NO_MEM;
    }

    if (xTaskCreate(discovery_worker, "lora_discovery", 4096, NULL, 8,
                    &s_worker_task) != pdPASS) {
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "Ready: MAC=%02X:%02X:%02X:%02X:%02X:%02X addr=%u SKU=%s",
             s_mac[0], s_mac[1], s_mac[2], s_mac[3], s_mac[4], s_mac[5],
             BOARD_MODBUS_SLAVE_ADDR, BOARD_PRODUCT_SKU);
    return ESP_OK;
}

bool lora_discovery_process_frame(const uint8_t *frame, size_t length)
{
    if (!frame || length < 2 || frame[0] != DISCOVERY_HEADER_0 ||
        frame[1] != DISCOVERY_HEADER_1) {
        return false;
    }

    if (length != DISCOVERY_REQUEST_FRAME_LEN ||
        frame[2] != DISCOVERY_VERSION ||
        frame[3] != DISCOVERY_COMMAND_REQUEST ||
        frame[4] != DISCOVERY_REQUEST_PAYLOAD_LEN) {
        ESP_LOGW(TAG, "Invalid discovery request format, length=%u",
                 (unsigned)length);
        return true;
    }

    const uint16_t received_crc =
        frame[length - 2] | ((uint16_t)frame[length - 1] << 8);
    const uint16_t calculated_crc = crc16_modbus(frame, length - 2);
    if (received_crc != calculated_crc) {
        ESP_LOGW(TAG, "Discovery request CRC error: got=0x%04X expected=0x%04X",
                 received_crc, calculated_crc);
        return true;
    }

    const uint16_t nonce = get_u16_be(&frame[5]);
    const uint8_t slot_count = frame[7];
    const uint16_t slot_duration_ms = get_u16_be(&frame[8]);
    if (slot_count == 0 || slot_duration_ms == 0) {
        ESP_LOGW(TAG, "Invalid slot settings: count=%u duration=%u ms",
                 slot_count, slot_duration_ms);
        return true;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_last_nonce_valid && nonce == s_last_nonce) {
        xSemaphoreGive(s_lock);
        ESP_LOGI(TAG, "Duplicate nonce 0x%04X ignored", nonce);
        return true;
    }

    s_last_nonce = nonce;
    s_last_nonce_valid = true;
    s_pending_request.nonce = nonce;
    s_pending_request.slot_count = slot_count;
    s_pending_request.slot_duration_ms = slot_duration_ms;
    ++s_pending_request.generation;
    xSemaphoreGive(s_lock);

    ESP_LOGI(TAG, "Discovery request accepted: nonce=0x%04X slots=%u x %u ms",
             nonce, slot_count, slot_duration_ms);
    xTaskNotifyGive(s_worker_task);
    return true;
}

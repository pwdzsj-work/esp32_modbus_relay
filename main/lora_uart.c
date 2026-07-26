#include "lora_uart.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/semphr.h"

#include <limits.h>

static const char *TAG = "LORA_UART";

static uart_port_t s_port = UART_NUM_MAX;
static SemaphoreHandle_t s_tx_mutex;
static bool s_initialized;

lora_uart_config_t lora_uart_get_default_config(void)
{
    return (lora_uart_config_t) {
        .port = LORA_UART_DEFAULT_PORT,
        .tx_gpio = LORA_UART_DEFAULT_TX_GPIO,
        .rx_gpio = LORA_UART_DEFAULT_RX_GPIO,
        .baud_rate = LORA_UART_DEFAULT_BAUD_RATE,
        .rx_buffer_size = LORA_UART_DEFAULT_RX_BUFFER_SIZE,
        .tx_buffer_size = LORA_UART_DEFAULT_TX_BUFFER_SIZE,
    };
}

static bool config_is_valid(const lora_uart_config_t *config)
{
    return config &&
           config->port >= UART_NUM_0 &&
           config->port < UART_NUM_MAX &&
           config->tx_gpio >= 0 &&
           config->rx_gpio >= 0 &&
           config->tx_gpio != config->rx_gpio &&
           config->baud_rate > 0 &&
           config->rx_buffer_size > UART_HW_FIFO_LEN(config->port) &&
           config->rx_buffer_size <= INT_MAX &&
           (config->tx_buffer_size == 0 ||
            (config->tx_buffer_size > UART_HW_FIFO_LEN(config->port) &&
             config->tx_buffer_size <= INT_MAX));
}

esp_err_t lora_uart_init(const lora_uart_config_t *config)
{
    if (!config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_initialized || uart_is_driver_installed(config->port)) {
        return ESP_ERR_INVALID_STATE;
    }

    SemaphoreHandle_t tx_mutex = xSemaphoreCreateMutex();
    if (!tx_mutex) {
        return ESP_ERR_NO_MEM;
    }

    const uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(config->port, &uart_config);
    if (err == ESP_OK) {
        err = uart_set_pin(config->port, config->tx_gpio, config->rx_gpio,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err == ESP_OK) {
        err = uart_driver_install(config->port,
                                  (int)config->rx_buffer_size,
                                  (int)config->tx_buffer_size,
                                  0, NULL, 0);
    }
    if (err != ESP_OK) {
        vSemaphoreDelete(tx_mutex);
        return err;
    }

    s_port = config->port;
    s_tx_mutex = tx_mutex;
    s_initialized = true;

    ESP_LOGI(TAG, "UART%d ready: TX=GPIO%d RX=GPIO%d, %d 8N1",
             config->port, config->tx_gpio, config->rx_gpio,
             config->baud_rate);
    return ESP_OK;
}

esp_err_t lora_uart_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    const uart_port_t port = s_port;
    SemaphoreHandle_t tx_mutex = s_tx_mutex;

    s_initialized = false;
    s_port = UART_NUM_MAX;
    s_tx_mutex = NULL;

    esp_err_t err = uart_driver_delete(port);
    vSemaphoreDelete(tx_mutex);
    return err;
}

esp_err_t lora_uart_send(const void *data, size_t length,
                         TickType_t timeout)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!data || length == 0 || length > INT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_tx_mutex, timeout) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    const int written = uart_write_bytes(s_port, data, length);
    esp_err_t err = written == (int)length
                        ? uart_wait_tx_done(s_port, timeout)
                        : ESP_FAIL;

    xSemaphoreGive(s_tx_mutex);
    return err;
}

int lora_uart_receive(void *data, size_t length, TickType_t timeout)
{
    if (!s_initialized || !data || length == 0 || length > INT_MAX) {
        return -1;
    }
    return uart_read_bytes(s_port, data, length, timeout);
}

esp_err_t lora_uart_get_buffered_length(size_t *length)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!length) {
        return ESP_ERR_INVALID_ARG;
    }
    return uart_get_buffered_data_len(s_port, length);
}

esp_err_t lora_uart_flush_input(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    return uart_flush_input(s_port);
}

bool lora_uart_is_initialized(void)
{
    return s_initialized;
}

uart_port_t lora_uart_get_port(void)
{
    return s_port;
}

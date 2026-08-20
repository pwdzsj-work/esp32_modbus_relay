#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Default connection for a UART transparent-transmission LoRa module.
 * UART2 is already used by RS485 in this project, so LoRa uses UART1.
 */
#define LORA_UART_DEFAULT_PORT           UART_NUM_1
#define LORA_UART_DEFAULT_TX_GPIO        16
#define LORA_UART_DEFAULT_RX_GPIO        17
#define LORA_UART_DEFAULT_BAUD_RATE      9600
#define LORA_UART_DEFAULT_RX_BUFFER_SIZE 1024
#define LORA_UART_DEFAULT_TX_BUFFER_SIZE 0

typedef struct {
    uart_port_t port;
    int tx_gpio;
    int rx_gpio;
    int baud_rate;
    size_t rx_buffer_size;
    size_t tx_buffer_size;
} lora_uart_config_t;

/* Fill a configuration structure with the defaults above. */
lora_uart_config_t lora_uart_get_default_config(void);

/* Install and configure the UART driver. Only one LoRa UART is supported. */
esp_err_t lora_uart_init(const lora_uart_config_t *config);

/* Release the UART driver installed by lora_uart_init(). */
esp_err_t lora_uart_deinit(void);

/*
 * Send one transparent data block and wait until it has left the UART.
 * timeout applies to the final UART transmission.
 */
esp_err_t lora_uart_send(const void *data, size_t length,
                         TickType_t timeout);

/*
 * Read up to length bytes. Returns the number of bytes read, or -1 when the
 * driver is not initialized/arguments are invalid.
 */
int lora_uart_receive(void *data, size_t length, TickType_t timeout);

/* Get the number of bytes currently waiting in the UART receive buffer. */
esp_err_t lora_uart_get_buffered_length(size_t *length);

/* Discard all bytes currently waiting in the UART receive buffer. */
esp_err_t lora_uart_flush_input(void);

bool lora_uart_is_initialized(void);
uart_port_t lora_uart_get_port(void);

#ifdef __cplusplus
}
#endif

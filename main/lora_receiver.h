#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Initialize the transparent LoRa UART and start the receive/logging task.
 * Uses the default UART settings declared in lora_uart.h.
 */
esp_err_t lora_receiver_init(void);

#ifdef __cplusplus
}
#endif

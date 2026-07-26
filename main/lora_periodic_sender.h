#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the automatic LoRa test transmission task. */
esp_err_t lora_periodic_sender_start(void);

#ifdef __cplusplus
}
#endif

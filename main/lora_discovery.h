#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the delayed LoRa MAC discovery response worker. */
esp_err_t lora_discovery_init(void);

/*
 * Process an AA 55 discovery frame. Returns true when the frame belongs to
 * the discovery protocol, including malformed discovery frames.
 */
bool lora_discovery_process_frame(const uint8_t *frame, size_t length);

#ifdef __cplusplus
}
#endif

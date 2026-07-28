#pragma once

#include "esp_err.h"

/* Configures SW3 for runtime detection. */
esp_err_t web_config_start_if_requested(void);

/*
 * Call periodically. Holding SW3 low for three seconds starts the Wi-Fi
 * access point and web server without resetting the ESP32.
 */
void web_config_poll(void);

#pragma once

#include "esp_err.h"

/* Configures SW3 for runtime detection. */
esp_err_t web_config_start_if_requested(void);

/*
 * Call periodically. Each three-second SW3 press toggles the Wi-Fi access
 * point and web server. SW3 must be released between toggle operations.
 */
void web_config_poll(void);

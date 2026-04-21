/**
 * @file wifi_transport.h
 * @brief WiFi station mode initialization and connection management.
 *
 * Directly ported from the working BITBot2 project.
 */
#pragma once

#include <stdbool.h>

/**
 * @brief Initialize WiFi in station mode and begin connecting.
 *        Uses CONFIG_ESP_WIFI_SSID / CONFIG_ESP_WIFI_PASSWORD from menuconfig.
 */
void wifi_init_sta(void);

/**
 * @brief Block until WiFi is connected or connection fails.
 */
void wifi_wait_for_connection(void);

/**
 * @brief Check current WiFi connection status.
 * @return true if connected and has IP address
 */
bool wifi_is_connected(void);

/**
 * @file pca9685_driver.h
 * @brief Low-level PCA9685 PWM driver abstraction.
 *
 * Provides a clean interface to set pulse widths on individual channels.
 * All I2C management is delegated to esp-idf-lib's i2cdev.
 */
#ifndef PCA9685_DRIVER_H
#define PCA9685_DRIVER_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the PCA9685 PWM controller.
 *
 * Sets up I2C subsystem via i2cdev, configures the PCA9685
 * descriptor, initializes the chip, and sets PWM frequency.
 *
 * @return ESP_OK on success
 */
esp_err_t pca9685_driver_init(void);

/**
 * @brief Set the pulse width on a specific PCA9685 channel.
 *
 * @param channel  PCA9685 output channel (0–15)
 * @param pulse_us Pulse width in microseconds (e.g., 500–2500 for servos)
 * @return ESP_OK on success
 */
esp_err_t pca9685_driver_set_pulse_us(uint8_t channel, float pulse_us);

/**
 * @brief Turn off all PWM outputs (set to 0).
 * @return ESP_OK on success
 */
esp_err_t pca9685_driver_all_off(void);

/**
 * @brief De-initialize the PCA9685 and release I2C resources.
 * @return ESP_OK on success
 */
esp_err_t pca9685_driver_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* PCA9685_DRIVER_H */

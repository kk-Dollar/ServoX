/**
 * @file pca9685_driver.c
 * @brief Low-level PCA9685 PWM driver implementation.
 *
 * Delegates all I2C bus management to esp-idf-lib's i2cdev layer.
 * We never call i2c_driver_install manually — the library handles it
 * internally during the first I2C transaction, avoiding port conflicts.
 */
#include "pca9685_driver.h"
#include "arm_config.h"
#include "esp_log.h"
#include <i2cdev.h>
#include <pca9685.h>

static const char *TAG = "PCA9685_DRV";
static i2c_dev_t s_dev;
static bool s_initialized = false;

esp_err_t pca9685_driver_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Initializing PCA9685 (addr=0x%02X, port=%d, SDA=%d, SCL=%d)",
             PCA9685_ADDR, ARM_I2C_PORT, ARM_I2C_SDA_PIN, ARM_I2C_SCL_PIN);

    /* Step 1: Initialize i2cdev subsystem (port mutexes) */
    ret = i2cdev_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2cdev_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Step 2: Fill in the device descriptor — i2cdev will install
       the I2C driver on the first actual bus transaction */
    ret = pca9685_init_desc(&s_dev, PCA9685_ADDR, ARM_I2C_PORT,
                            ARM_I2C_SDA_PIN, ARM_I2C_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "pca9685_init_desc failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Override the library's default 1 MHz I2C clock to our stable 100 kHz */
    s_dev.cfg.master.clk_speed = ARM_I2C_FREQ_HZ;

    /* Step 3: Initialize the PCA9685 chip (triggers first I2C transaction) */
    ret = pca9685_init(&s_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "pca9685_init failed: %s (check wiring/power/address)",
                 esp_err_to_name(ret));
        return ret;
    }

    /* Step 4: Set 50 Hz PWM for standard analog servos */
    ret = pca9685_set_pwm_frequency(&s_dev, PCA9685_PWM_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "pca9685_set_pwm_frequency failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "PCA9685 initialized successfully (PWM %d Hz)", PCA9685_PWM_FREQ_HZ);
    return ESP_OK;
}

esp_err_t pca9685_driver_set_pulse_us(uint8_t channel, float pulse_us)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    /* Convert microseconds → 12-bit PWM tick count
       Period = 1/50 Hz = 20000 µs.  Resolution = 4096 ticks / 20000 µs */
    float period_us = 1000000.0f / (float)PCA9685_PWM_FREQ_HZ;
    uint16_t tick = (uint16_t)(pulse_us * (float)PCA9685_RESOLUTION / period_us);

    return pca9685_set_pwm_value(&s_dev, channel, tick);
}

esp_err_t pca9685_driver_all_off(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    for (int i = 0; i < 16; i++) {
        pca9685_set_pwm_value(&s_dev, i, 0);
    }
    return ESP_OK;
}

esp_err_t pca9685_driver_deinit(void)
{
    if (!s_initialized) return ESP_OK;

    pca9685_driver_all_off();
    s_initialized = false;
    return i2cdev_done();
}

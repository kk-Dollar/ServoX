/**
 * @file app_main.c
 * @brief Application entry point for the 4-DOF robotic arm.
 *
 * Boot sequence follows a strict order to ensure safety:
 *   1. NVS (required for WiFi)
 *   2. Servo controller (PCA9685 + home position)
 *   3. WiFi connection
 *   4. micro-ROS agent connection
 *
 * Hardware is initialized BEFORE networking so the arm
 * is in a known safe position before any commands arrive.
 */
#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "arm_config.h"
#include "servo_controller.h"
#include "wifi_transport.h"
#include "microros_node.h"

static const char *TAG = "APP_MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ServoX — 3-DOF Robotic Arm");
    ESP_LOGI(TAG, "  Joints: %d | Control: %d Hz",
             ARM_NUM_JOINTS, CONTROL_LOOP_HZ);
    ESP_LOGI(TAG, "  micro-ROS over WiFi");
    ESP_LOGI(TAG, "========================================");

    /* ── Step 1: Initialize NVS (required for WiFi) ── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* ── Step 2: Initialize servo hardware (safe home position) ── */
    ESP_LOGI(TAG, "Initializing servo controller...");
    ret = servo_controller_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CRITICAL: Servo controller init failed: %s",
                 esp_err_to_name(ret));
        ESP_LOGE(TAG, "Check: PCA9685 wiring, I2C address, 5V servo power");
        ESP_LOGE(TAG, "System halted.");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }
    ESP_LOGI(TAG, "Servo controller: OK (arm at home position)");

    /* ── Step 3: Connect to WiFi ── */
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();

    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    wifi_wait_for_connection();

    if (!wifi_is_connected()) {
        ESP_LOGE(TAG, "WiFi not connected! Cannot start micro-ROS.");
        ESP_LOGE(TAG, "Check: SSID/password in menuconfig, router powered on");
        ESP_LOGE(TAG, "System halted. Arm holding home position.");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }
    ESP_LOGI(TAG, "WiFi: OK");

    /* ── Step 4: Connect to micro-ROS agent ── */
    ESP_LOGI(TAG, "Connecting to micro-ROS agent...");
    int mr_ret = microros_start();
    if (mr_ret != ESP_OK) {
        ESP_LOGE(TAG, "micro-ROS agent not reachable.");
        ESP_LOGE(TAG, "Check: Agent IP/port in menuconfig, agent running");
        ESP_LOGE(TAG, "Arm holding home position. Restart to retry.");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }
    ESP_LOGI(TAG, "micro-ROS: OK");

    /* ── Boot complete ── */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  System Ready!");
    ESP_LOGI(TAG, "  SUB: %s", TOPIC_ARM_COMMAND);
    ESP_LOGI(TAG, "  PUB: %s", TOPIC_ARM_STATE);
    ESP_LOGI(TAG, "========================================");

    /* app_main returns; FreeRTOS tasks handle everything:
       - servo_ctrl task: 50 Hz interpolation loop
       - microros task: ROS executor + state publishing */
}
/**
 * @file servo_controller.c
 * @brief Velocity-limited servo interpolation engine.
 *
 * Runs a dedicated FreeRTOS task at 50 Hz. Each tick:
 *   1. Computes the maximum step each joint can take (speed * dt)
 *   2. Advances current_angle toward target_angle by at most that step
 *   3. Converts the current angle to a pulse width via linear interpolation
 *   4. Commands the PCA9685 driver
 *
 * This guarantees smooth, jerk-free motion regardless of how large
 * the commanded angle jump is.
 */
#include "servo_controller.h"
#include "pca9685_driver.h"
#include "safety_layer.h"
#include "arm_config.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "SERVO_CTRL";

/* ── Internal state ────────────────────────────────────────────────────── */
static float s_current[ARM_NUM_JOINTS];
static float s_target[ARM_NUM_JOINTS];
static TaskHandle_t s_task_handle = NULL;

/* ── Angle → Pulse conversion ──────────────────────────────────────────── */
static float angle_to_pulse_us(int joint, float angle_deg)
{
    const joint_config_t *cfg = &ARM_JOINT_CONFIG[joint];

    /* Linear interpolation between [angle_min, angle_max] → [pulse_min, pulse_max] */
    float ratio = (angle_deg - cfg->angle_min_deg)
                / (cfg->angle_max_deg - cfg->angle_min_deg);

    return cfg->pulse_min_us + ratio * (cfg->pulse_max_us - cfg->pulse_min_us);
}

/* ── 50 Hz control loop ───────────────────────────────────────────────── */
static void control_loop_task(void *arg)
{
    const float dt = (float)CONTROL_LOOP_MS / 1000.0f;
    int log_counter = 0;

    ESP_LOGI(TAG, "Control loop started (%d Hz, dt=%.3fs)", CONTROL_LOOP_HZ, dt);

    while (1) {
        for (int j = 0; j < ARM_NUM_JOINTS; j++) {
            float diff = s_target[j] - s_current[j];

            if (fabsf(diff) < 0.01f) {
                /* Close enough — snap to target to avoid float drift */
                s_current[j] = s_target[j];
            } else {
                /* Maximum step this tick based on configured velocity */
                float max_step = ARM_JOINT_CONFIG[j].max_speed_dps * dt;
                float step = fminf(fabsf(diff), max_step);
                s_current[j] += (diff > 0.0f) ? step : -step;
            }

            /* Drive the servo */
            float pulse = angle_to_pulse_us(j, s_current[j]);
            pca9685_driver_set_pulse_us(ARM_JOINT_CONFIG[j].pca_channel, pulse);
        }

        /* Periodic state logging (~every 2 seconds) */
        if (++log_counter >= (CONTROL_LOOP_HZ * 2)) {
            ESP_LOGI(TAG, "POS [%.1f, %.1f, %.1f, %.1f] -> TGT [%.1f, %.1f, %.1f, %.1f]",
                     s_current[0], s_current[1], s_current[2], s_current[3],
                     s_target[0],  s_target[1],  s_target[2],  s_target[3]);
            log_counter = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_MS));
    }
}

/* ── Public API ────────────────────────────────────────────────────────── */

esp_err_t servo_controller_init(void)
{
    ESP_LOGI(TAG, "Initializing servo controller (%d joints)", ARM_NUM_JOINTS);

    /* Initialize PCA9685 hardware */
    esp_err_t ret = pca9685_driver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Set all joints to their configured home positions */
    for (int j = 0; j < ARM_NUM_JOINTS; j++) {
        float home = ARM_JOINT_CONFIG[j].home_deg;
        s_current[j] = home;
        s_target[j]  = home;

        float pulse = angle_to_pulse_us(j, home);
        pca9685_driver_set_pulse_us(ARM_JOINT_CONFIG[j].pca_channel, pulse);
    }
    ESP_LOGI(TAG, "Home position set: [%.1f, %.1f, %.1f, %.1f]",
             s_current[0], s_current[1], s_current[2], s_current[3]);

    /* Start control loop task */
    BaseType_t xret = xTaskCreate(control_loop_task, "servo_ctrl",
                                  3072, NULL, 7, &s_task_handle);
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control loop task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t servo_controller_set_target(int joint, float angle_deg)
{
    if (joint < 0 || joint >= ARM_NUM_JOINTS) return ESP_ERR_INVALID_ARG;

    /* Safety clamping */
    s_target[joint] = safety_clamp_angle(joint, angle_deg);
    return ESP_OK;
}

esp_err_t servo_controller_set_all_targets(const float *angles_deg)
{
    if (!angles_deg) return ESP_ERR_INVALID_ARG;

    for (int j = 0; j < ARM_NUM_JOINTS; j++) {
        s_target[j] = safety_clamp_angle(j, angles_deg[j]);
    }
    return ESP_OK;
}

float servo_controller_get_angle(int joint)
{
    if (joint < 0 || joint >= ARM_NUM_JOINTS) return 0.0f;
    return s_current[joint];
}

void servo_controller_get_all_angles(float *out_angles)
{
    if (!out_angles) return;
    memcpy(out_angles, s_current, sizeof(float) * ARM_NUM_JOINTS);
}

esp_err_t servo_controller_deinit(void)
{
    if (s_task_handle) {
        vTaskDelete(s_task_handle);
        s_task_handle = NULL;
    }
    return pca9685_driver_deinit();
}

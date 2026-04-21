/**
 * @file safety_layer.c
 * @brief Safety enforcement for joint commands.
 *
 * Pure logic — no hardware dependencies. Easy to unit test.
 */
#include "safety_layer.h"
#include "arm_config.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "SAFETY";

/* Timestamp of last valid command (microseconds) */
static int64_t s_last_cmd_us = 0;

float safety_clamp_angle(int joint, float deg_in)
{
    if (joint < 0 || joint >= ARM_NUM_JOINTS) {
        ESP_LOGW(TAG, "Invalid joint index %d", joint);
        return 0.0f;
    }

    const joint_config_t *cfg = &ARM_JOINT_CONFIG[joint];

    if (deg_in < cfg->angle_min_deg) return cfg->angle_min_deg;
    if (deg_in > cfg->angle_max_deg) return cfg->angle_max_deg;
    return deg_in;
}

float safety_rad_to_clamped_deg(int joint, float rad_in)
{
    float deg = rad_in * (180.0f / (float)M_PI);
    return safety_clamp_angle(joint, deg);
}

void safety_mark_command_received(void)
{
    s_last_cmd_us = esp_timer_get_time();
}

bool safety_is_command_timeout(void)
{
    /* Before first command, consider it "timed out" so arm holds home */
    if (s_last_cmd_us == 0) return true;

    int64_t elapsed_ms = (esp_timer_get_time() - s_last_cmd_us) / 1000;
    return elapsed_ms > CMD_TIMEOUT_MS;
}

bool safety_validate_msg_size(size_t received_size)
{
    if (received_size < ARM_NUM_JOINTS) {
        ESP_LOGW(TAG, "Rejected command: expected %d elements, got %zu",
                 ARM_NUM_JOINTS, received_size);
        return false;
    }
    return true;
}

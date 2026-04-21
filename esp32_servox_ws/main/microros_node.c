/**
 * @file microros_node.c
 * @brief micro-ROS node for 4-DOF robotic arm control.
 *
 * Architecture follows the proven BITBot2 pattern:
 *   - microros_start() handles agent discovery with retries (blocking)
 *   - On success, spawns microros_task which runs the executor loop
 *
 * Communication:
 *   SUB: /arm/command  [Float32MultiArray, 4 floats in radians]
 *   PUB: /arm/state    [Float32MultiArray, 4 floats in radians]
 *
 * The node does NOT directly drive servos. It converts radians → degrees,
 * validates through the safety layer, and sets targets on the servo controller.
 */
#include "microros_node.h"
#include "servo_controller.h"
#include "safety_layer.h"
#include "arm_config.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <rmw_microros/ping.h>
#include <rmw_microros/rmw_microros.h>
#include <uros_network_interfaces.h>

static const char *TAG = "MICROROS";

/* ── micro-ROS objects ─────────────────────────────────────────────────── */
static rclc_support_t  support;
static rcl_node_t      node;
static rcl_subscription_t cmd_sub;
static rcl_publisher_t    state_pub;
static rclc_executor_t    executor;

/* ── Messages ──────────────────────────────────────────────────────────── */
static std_msgs__msg__Float32MultiArray cmd_msg;
static float cmd_data[ARM_NUM_JOINTS];

static std_msgs__msg__Float32MultiArray state_msg;
static float state_data[ARM_NUM_JOINTS];

static volatile bool s_microros_connected = false;

/* ── Command callback ──────────────────────────────────────────────────── */
static void cmd_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

    /* Reject malformed messages */
    if (!safety_validate_msg_size(msg->data.size)) return;

    /* Convert radians → clamped degrees, set targets */
    float targets_deg[ARM_NUM_JOINTS];
    for (int j = 0; j < ARM_NUM_JOINTS; j++) {
        targets_deg[j] = safety_rad_to_clamped_deg(j, msg->data.data[j]);
    }

    servo_controller_set_all_targets(targets_deg);
    safety_mark_command_received();
}

/* ── Main executor task ────────────────────────────────────────────────── */
static void microros_task(void *arg)
{
    ESP_LOGI(TAG, "micro-ROS task starting");

    /* --- Subscriber: /arm/command (reliable for guaranteed delivery) --- */
    rclc_subscription_init_default(
        &cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        TOPIC_ARM_COMMAND);

    /* Initialize command message buffer */
    std_msgs__msg__Float32MultiArray__init(&cmd_msg);
    cmd_msg.data.capacity = ARM_NUM_JOINTS;
    cmd_msg.data.size     = 0;
    cmd_msg.data.data     = cmd_data;

    /* --- Publisher: /arm/state (reliable) --- */
    rclc_publisher_init_default(
        &state_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        TOPIC_ARM_STATE);

    std_msgs__msg__Float32MultiArray__init(&state_msg);
    state_msg.data.capacity = ARM_NUM_JOINTS;
    state_msg.data.size     = ARM_NUM_JOINTS;
    state_msg.data.data     = state_data;

    /* --- Executor --- */
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg,
                                   &cmd_callback, ON_NEW_DATA);

    ESP_LOGI(TAG, "Listening on %s, publishing on %s",
             TOPIC_ARM_COMMAND, TOPIC_ARM_STATE);

    s_microros_connected = true;

    int64_t last_ping_us  = 0;
    int64_t last_state_us = esp_timer_get_time();
    int     ping_fails    = 0;
    const int64_t state_period_us = 1000000LL / STATE_PUB_HZ;

    while (1) {
        int64_t now_us = esp_timer_get_time();

        /* ── Ping agent every 2 seconds ── */
        if ((now_us - last_ping_us) >= 2000000LL) {
            rmw_ret_t ping_ret = rmw_uros_ping_agent(100, 3);
            if (ping_ret != RMW_RET_OK) {
                ping_fails++;
                if (ping_fails >= 10 && s_microros_connected) {
                    ESP_LOGW(TAG, "Agent ping failed (count %d)", ping_fails);
                }
            } else {
                ping_fails = 0;
                s_microros_connected = true;
            }
            last_ping_us = now_us;
        }

        /* ── Spin executor (non-blocking) ── */
        rclc_executor_spin_some(&executor, 0);

        /* ── Publish state at STATE_PUB_HZ ── */
        if ((now_us - last_state_us) >= state_period_us) {
            float angles_deg[ARM_NUM_JOINTS];
            servo_controller_get_all_angles(angles_deg);

            /* Convert degrees → radians for ROS */
            for (int j = 0; j < ARM_NUM_JOINTS; j++) {
                state_data[j] = angles_deg[j] * ((float)M_PI / 180.0f);
            }

            rcl_publish(&state_pub, &state_msg, NULL);
            last_state_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ── Public API ────────────────────────────────────────────────────────── */

int microros_start(void)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rcl_ret_t ret;
    int retry_count = 0;
    const int MAX_RETRIES = 5;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Waiting for micro-ROS agent");
    ESP_LOGI(TAG, "  IP: %s, Port: %s", CONFIG_MICRO_ROS_AGENT_IP,
             CONFIG_MICRO_ROS_AGENT_PORT);
    ESP_LOGI(TAG, "========================================");

    while (retry_count < MAX_RETRIES) {
        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        ret = rcl_init_options_init(&init_options, allocator);
        if (ret != RCL_RET_OK) {
            ESP_LOGW(TAG, "[%d/%d] Failed to init options",
                     retry_count + 1, MAX_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(500));
            retry_count++;
            continue;
        }

        rmw_init_options_t *rmw_options =
            rcl_init_options_get_rmw_init_options(&init_options);
        ret = rmw_uros_options_set_udp_address(
            CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);
        if (ret != RMW_RET_OK) {
            ESP_LOGW(TAG, "[%d/%d] Failed to set UDP address",
                     retry_count + 1, MAX_RETRIES);
            rcl_init_options_fini(&init_options);
            vTaskDelay(pdMS_TO_TICKS(500));
            retry_count++;
            continue;
        }

        ESP_LOGI(TAG, "[%d/%d] Attempting connection to agent...",
                 retry_count + 1, MAX_RETRIES);

        ret = rclc_support_init_with_options(&support, 0, NULL,
                                             &init_options, &allocator);
        if (ret != RCL_RET_OK) {
            ESP_LOGW(TAG, "[%d/%d] Agent not available (code %ld)",
                     retry_count + 1, MAX_RETRIES, (long)ret);
            rcl_init_options_fini(&init_options);
            vTaskDelay(pdMS_TO_TICKS(500));
            retry_count++;
            continue;
        }
        ESP_LOGI(TAG, "Support initialized successfully");

        ret = rclc_node_init_default(&node, MICROROS_NODE_NAME, "", &support);
        if (ret != RCL_RET_OK) {
            ESP_LOGW(TAG, "Failed to init node (attempt %d/%d)",
                     retry_count + 1, MAX_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(500));
            retry_count++;
            continue;
        }
        ESP_LOGI(TAG, "Node created: %s", MICROROS_NODE_NAME);

        ESP_LOGI(TAG, "Agent reachable, starting micro-ROS task");
        s_microros_connected = true;
        xTaskCreate(microros_task, "microros", 8192, NULL, 6, NULL);
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Failed to connect to agent after %d attempts", MAX_RETRIES);
    return ESP_FAIL;
}

bool microros_is_connected(void)
{
    return s_microros_connected;
}

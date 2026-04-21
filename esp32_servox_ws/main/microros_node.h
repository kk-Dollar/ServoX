/**
 * @file microros_node.h
 * @brief micro-ROS communication node for the 4-DOF arm.
 *
 * Subscribes to /arm/command (Float32MultiArray, radians)
 * Publishes to /arm/state (Float32MultiArray, radians)
 */
#pragma once

#include <stdbool.h>

/**
 * @brief Connect to micro-ROS agent and start the executor task.
 *
 * Blocks with retries until the agent is found or MAX_RETRIES is exhausted.
 * On success, spawns a FreeRTOS task that handles subscriptions and publishing.
 *
 * @return ESP_OK on success, ESP_FAIL on failure
 */
int microros_start(void);

/**
 * @brief Check if the micro-ROS agent session is currently active.
 * @return true if connected
 */
bool microros_is_connected(void);

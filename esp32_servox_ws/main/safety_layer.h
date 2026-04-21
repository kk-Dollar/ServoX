/**
 * @file safety_layer.h
 * @brief Safety layer for command validation, clamping, and timeout detection.
 *
 * Sits between the ROS subscriber and the servo controller.
 * Enforces joint limits, rejects malformed commands, and detects
 * communication timeouts to hold the arm in a safe position.
 */
#ifndef SAFETY_LAYER_H
#define SAFETY_LAYER_H

#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Validate and clamp a joint command (in degrees).
 *
 * @param joint   Joint index (0 .. ARM_NUM_JOINTS-1)
 * @param deg_in  Requested angle in degrees
 * @return        Clamped angle within the joint's mechanical limits
 */
float safety_clamp_angle(int joint, float deg_in);

/**
 * @brief Convert radians to degrees and clamp to joint limits.
 *
 * @param joint   Joint index
 * @param rad_in  Requested angle in radians
 * @return        Clamped angle in degrees
 */
float safety_rad_to_clamped_deg(int joint, float rad_in);

/**
 * @brief Record the timestamp of the most recent valid command.
 */
void safety_mark_command_received(void);

/**
 * @brief Check whether the command stream has timed out.
 *
 * @return true if no valid command has been received within CMD_TIMEOUT_MS
 */
bool safety_is_command_timeout(void);

/**
 * @brief Validate that a Float32MultiArray has the expected size.
 *
 * @param received_size Number of elements in the array
 * @return true if valid (== ARM_NUM_JOINTS), false otherwise
 */
bool safety_validate_msg_size(size_t received_size);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_LAYER_H */

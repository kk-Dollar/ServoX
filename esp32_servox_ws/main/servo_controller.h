/**
 * @file servo_controller.h
 * @brief Smooth servo motion controller with velocity limiting.
 *
 * Owns the control loop running at 50 Hz. Each tick:
 *   1. Reads the latest target angles
 *   2. Applies per-joint velocity limiting
 *   3. Advances current angles by at most max_speed * dt
 *   4. Sends PWM commands to the PCA9685
 *
 * The caller only needs to set target angles; the controller
 * handles all interpolation and smoothing internally.
 */
#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "esp_err.h"
#include "arm_config.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the servo controller.
 *
 * - Initializes the PCA9685 driver
 * - Moves all joints to their home positions
 * - Starts the 50 Hz control loop FreeRTOS task
 *
 * @return ESP_OK on success
 */
esp_err_t servo_controller_init(void);

/**
 * @brief Set the target angle for a specific joint (in degrees).
 *
 * The controller will smoothly interpolate toward this target
 * at the joint's configured max_speed_dps.
 *
 * @param joint     Joint index (0 .. ARM_NUM_JOINTS-1)
 * @param angle_deg Target angle in degrees (will be clamped by safety layer)
 * @return ESP_OK on success
 */
esp_err_t servo_controller_set_target(int joint, float angle_deg);

/**
 * @brief Set all joint targets at once (in degrees).
 *
 * @param angles_deg Array of ARM_NUM_JOINTS target angles
 * @return ESP_OK on success
 */
esp_err_t servo_controller_set_all_targets(const float *angles_deg);

/**
 * @brief Get the current estimated angle for a joint (in degrees).
 *
 * @param joint Joint index
 * @return Current interpolated angle
 */
float servo_controller_get_angle(int joint);

/**
 * @brief Get all current angles (in degrees).
 *
 * @param out_angles Output array (must be ARM_NUM_JOINTS elements)
 */
void servo_controller_get_all_angles(float *out_angles);

/**
 * @brief De-initialize: stop control task, turn off PWM.
 * @return ESP_OK on success
 */
esp_err_t servo_controller_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_CONTROLLER_H */

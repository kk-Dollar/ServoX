/**
 * @file arm_config.h
 * @brief Central configuration for the 4-DOF robotic arm.
 *
 * All tunable parameters live here. No magic numbers elsewhere.
 * Changing a calibration value should never require editing control logic.
 */
#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ═══════════════════════════════════════════════════════════════════════════
 *  I2C Bus Configuration
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ARM_I2C_PORT          0
#define ARM_I2C_SDA_PIN       21
#define ARM_I2C_SCL_PIN       22
#define ARM_I2C_FREQ_HZ       100000   /* 100 kHz — reliable for PCA9685 over jumper wires */

/* ═══════════════════════════════════════════════════════════════════════════
 *  PCA9685 PWM Driver
 * ═══════════════════════════════════════════════════════════════════════════ */
#define PCA9685_ADDR          0x40
#define PCA9685_PWM_FREQ_HZ   50       /* Standard servo frequency */
#define PCA9685_RESOLUTION    4096     /* 12-bit PWM */

/* ═══════════════════════════════════════════════════════════════════════════
 *  Number of Joints
 * ═══════════════════════════════════════════════════════════════════════════ */
#define ARM_NUM_JOINTS        4

/* ═══════════════════════════════════════════════════════════════════════════
 *  Per-Joint Configuration
 *
 *  Each joint has:
 *    - PCA9685 channel number
 *    - Mechanical angle limits (degrees)
 *    - PWM pulse width calibration (microseconds)
 *    - Safe neutral position (degrees)
 *    - Maximum angular velocity (degrees/second)
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    uint8_t  pca_channel;       /* PCA9685 output channel (0–15)              */
    float    angle_min_deg;     /* Mechanical lower limit                     */
    float    angle_max_deg;     /* Mechanical upper limit                     */
    float    pulse_min_us;      /* Pulse width at angle_min_deg               */
    float    pulse_max_us;      /* Pulse width at angle_max_deg               */
    float    park_deg;          /* Resting position when powered off          */
    float    home_deg;          /* Safe startup position                      */
    float    max_speed_dps;     /* Maximum velocity in degrees per second     */
} joint_config_t;

/**
 * Default joint configurations.
 *
 * Pulse calibration:
 *   SG90 servo typical range is 500 µs → 2500 µs for 0° → 180°.
 *   Adjust per-joint after physical measurement.
 *
 * Speed:
 *   15 deg/s gives smooth, non-aggressive motion.
 *   Reduce further for shoulder/elbow if the arm is heavy.
 */
static const joint_config_t ARM_JOINT_CONFIG[ARM_NUM_JOINTS] = {
    /* ch   min°   max°    pulse_min  pulse_max   park°  home°  speed_dps */
    
    /* Base: 0 to 180 degrees -> Uses the full 500 to 2500 us range */
    {  0,   -90f, 90.0f,  500.0f,   2500.0f,   90.0f, 90.0f,  15.0f },  
    
    /* Shoulder: 0 to 30 degrees -> 500 to 833 us (falls to ~30 due to gravity) */
    {  1,   0.0f,  45.0f,  500.0f,    833.3f,   30.0f,  5.0f,  10.0f },  
    
    /* Elbow: 0 to 90 degrees -> 500 to 1500 us (falls to ~90 due to gravity) */
    {  2,   0.0f,  90.0f,  500.0f,   1500.0f,   90.0f,  0.0f,  15.0f },  
    
    /* Gripper: 6 to 45 degrees -> 500 to 1000 us (6 is fully closed) */
    {  3,   6.0f,  45.0f,  500.0f,   1000.0f,    6.0f,  6.0f,  15.0f },  
};

/* ═══════════════════════════════════════════════════════════════════════════
 *  Control Loop Timing
 * ═══════════════════════════════════════════════════════════════════════════ */
#define CONTROL_LOOP_HZ       50       /* 50 Hz = 20 ms period                */
#define CONTROL_LOOP_MS       (1000 / CONTROL_LOOP_HZ)

/* ═══════════════════════════════════════════════════════════════════════════
 *  Safety Parameters
 * ═══════════════════════════════════════════════════════════════════════════ */
#define CMD_TIMEOUT_MS        500      /* Hold position if no command for this long */

/* ═══════════════════════════════════════════════════════════════════════════
 *  micro-ROS Topics
 * ═══════════════════════════════════════════════════════════════════════════ */
#define TOPIC_ARM_COMMAND     "/arm/command"
#define TOPIC_ARM_STATE       "/arm/state"
#define MICROROS_NODE_NAME    "servox_arm_node"

/* ═══════════════════════════════════════════════════════════════════════════
 *  State Publisher Rate (can be slower than control loop)
 * ═══════════════════════════════════════════════════════════════════════════ */
#define STATE_PUB_HZ          25       /* 25 Hz state publishing               */

#ifdef __cplusplus
}
#endif

#endif /* ARM_CONFIG_H */

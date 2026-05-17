#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define PWM_SERIAL_DEBUG_ONLY 1

#define PWM_SERIAL_DEBUG_MIN_US 1000u
#define PWM_SERIAL_DEBUG_MAX_US 2000u
#define PWM_SERIAL_DEBUG_NEUTRAL_US 1500u

#define PIN_ENCODER_AB 2u
#define PIN_PWM_ESC 9u

#define CONTROL_PERIOD_MS 20
#define CONTROL_DT_S 0.02f
#define PI_F 3.14159265358979323846f

#define ENCODER_COUNTS_PER_REV 127.0f
#define WHEEL_RADIUS_M 0.055f
#define GEAR_RATIO_SHAFT_PER_WHEEL 4.0f

#define REF_SPEED_MPS_DEFAULT 10.0f

#define REF_SPEED_TOPIC "ref_speed"
#define MEASURED_SPEED_TOPIC "measured_speed"
#define ENCODER_COUNT_TOPIC "encoder_count"
#define CONTROL_FORCE_TOPIC "control_force"

#define MICRO_ROS_AGENT_PING_TIMEOUT_MS 1000
#define MICRO_ROS_AGENT_PING_ATTEMPTS 120u

/* true: micro-ROS; false: USB serial telemetry only */
#define ROS_MODE false

#define CTRL_FORCE_MIN_N -18.5f
#define CTRL_FORCE_MAX_N 20.0f

#if !PWM_SERIAL_DEBUG_ONLY
#define ESC_MIN_US 1100u
#define ESC_NEUTRAL_US 1300u
#define ESC_MAX_US 1380u
#endif

#define ESC_DEADZONE_LOW_US 1300u
#define ESC_DEADZONE_HIGH_US 1300u

#define ESC_PWM_FREQUENCY_HZ 50u
#define ESC_PWM_PERIOD_US (1000000u / ESC_PWM_FREQUENCY_HZ)
#define ESC_PWM_INVERT_OUTPUT 1

#define SERIAL_LINE_BUF_SIZE 32u
#define USB_SERIAL_BOOT_DELAY_MS 1200u
#define USB_SERIAL_WAIT_STEP_MS 100u

#endif

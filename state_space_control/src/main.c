
#include <math.h>
#include <stdio.h>

#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico_uart_transports.h"
#include "pwm_esc.h"
#include "quadrature_encoder.pio.h"
#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "state_space_control.h"
#include "std_msgs/msg/float64.h"

// Base pin for encoder channel A. Channel B must be connected to the next pin.
#define PIN_AB 2
#define PIN_PWM_ESC 6

#define CONTROL_PERIOD_MS 20
#define CONTROL_DT_S 0.02f
#define PI_F 3.14159265358979323846f

// Tune these constants for your drivetrain/encoder.
#define ENCODER_COUNTS_PER_REV 127.0f
#define WHEEL_RADIUS_M 0.055f

// Reference speed [m/s]. Will be subscribed from ROS 2 if available.
#define REF_SPEED_MPS_DEFAULT 0.1f
#define REF_SPEED_TOPIC "ref_speed"
#define MICRO_ROS_AGENT_PING_TIMEOUT_MS 1000
#define MICRO_ROS_AGENT_PING_ATTEMPTS 120u

// Toggle mode: true = serial debug output, false = micro-ROS node mode.
#define DEBUG true

// Controller output from Simulink saturation block: [-10, +10] N.
#define CTRL_FORCE_MIN_N -10.0f
#define CTRL_FORCE_MAX_N 10.0f

// ESC pulse widths in microseconds (calibrate for your ESC).
#define ESC_MIN_US 1400u
#define ESC_NEUTRAL_US 1500u
#define ESC_MAX_US 1700u

static PIO g_pio = pio0;
static const uint g_sm = 0;
static pwm_esc_t g_esc;
static struct repeating_timer g_control_timer;

// Reference speed [m/s] - shared between ROS 2 subscription and control loop
static volatile float g_ref_speed_mps = REF_SPEED_MPS_DEFAULT;

static volatile int32_t g_prev_encoder_count = 0;
static volatile int32_t g_encoder_count = 0;
static volatile float g_measured_speed_mps = 0.0f;
static volatile float g_control_force_n = 0.0f;
static volatile uint16_t g_pwm_us = ESC_NEUTRAL_US;

// Timing instrumentation for interrupt validation
static uint32_t g_last_callback_us = 0;
static volatile uint32_t g_last_interval_us = 0;
static volatile uint32_t g_callback_count = 0;
static volatile uint32_t g_min_interval_us = 0xFFFFFFFFU;
static volatile uint32_t g_max_interval_us = 0;

static rcl_allocator_t g_allocator;
static rclc_support_t g_support;
static rcl_node_t g_node;
static rcl_subscription_t g_ref_speed_subscription;
static rclc_executor_t g_executor;
static std_msgs__msg__Float64 g_ref_speed_msg;
static bool g_micro_ros_connected = false;

static void ref_speed_subscription_callback(const void *msgin) {
    const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msgin;
    uint32_t irq_state = save_and_disable_interrupts();
    g_ref_speed_mps = (float)msg->data;
    restore_interrupts(irq_state);
}

static bool micro_ros_init_subscription(void) {
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    if (rmw_uros_ping_agent(MICRO_ROS_AGENT_PING_TIMEOUT_MS, MICRO_ROS_AGENT_PING_ATTEMPTS) != RCL_RET_OK) {
        return false;
    }

    g_allocator = rcl_get_default_allocator();
    g_support = (rclc_support_t){0};
    g_node = rcl_get_zero_initialized_node();
    g_ref_speed_subscription = rcl_get_zero_initialized_subscription();
    g_executor = rclc_executor_get_zero_initialized_executor();
    g_ref_speed_msg = (std_msgs__msg__Float64){0};

    if (rclc_support_init(&g_support, 0, NULL, &g_allocator) != RCL_RET_OK) {
        return false;
    }

    if (rclc_node_init_default(&g_node, "ssc", "", &g_support) != RCL_RET_OK) {
        return false;
    }

    if (rclc_subscription_init_default(
            &g_ref_speed_subscription,
            &g_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
            REF_SPEED_TOPIC) != RCL_RET_OK) {
        return false;
    }

    if (rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator) != RCL_RET_OK) {
        return false;
    }

    if (rclc_executor_add_subscription(
            &g_executor,
            &g_ref_speed_subscription,
            &g_ref_speed_msg,
            ref_speed_subscription_callback,
            ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }

    g_micro_ros_connected = true;
    return true;
}

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline uint16_t force_to_pwm_us(float force_n) {
    float normalized = clampf(force_n, CTRL_FORCE_MIN_N, CTRL_FORCE_MAX_N) / CTRL_FORCE_MAX_N;

    if (normalized >= 0.0f) {
        float span = (float)ESC_MAX_US - (float)ESC_NEUTRAL_US;
        return (uint16_t)((float)ESC_NEUTRAL_US + normalized * span);
    }

    {
        float span = (float)ESC_NEUTRAL_US - (float)ESC_MIN_US;
        return (uint16_t)((float)ESC_NEUTRAL_US + normalized * span);
    }
}

static bool control_timer_callback(struct repeating_timer *t) {
    (void)t;

    // Measure timing
    uint32_t now_us = time_us_32();
    if (g_last_callback_us != 0) {
        uint32_t interval_us = now_us - g_last_callback_us;
        g_last_interval_us = interval_us;

        if (interval_us < g_min_interval_us) {
            g_min_interval_us = interval_us;
        }
        if (interval_us > g_max_interval_us) {
            g_max_interval_us = interval_us;
        }
    }
    g_last_callback_us = now_us;
    g_callback_count++;

    const float wheel_circumference = 2.0f * PI_F * WHEEL_RADIUS_M;
    const float meters_per_count = wheel_circumference / ENCODER_COUNTS_PER_REV;

    int32_t encoder_now = quadrature_encoder_get_count(g_pio, g_sm);
    int32_t delta = encoder_now - g_prev_encoder_count;
    g_prev_encoder_count = encoder_now;

    float y_mps = ((float)delta * meters_per_count) / CONTROL_DT_S;

    rtU.r = (real_T)g_ref_speed_mps;
    rtU.y = (real_T)y_mps;
    state_space_control_step();

    float uc_n = (float)rtY.uc;
    uint16_t pwm_us = force_to_pwm_us(uc_n);
    pwm_esc_set_speed_us(&g_esc, pwm_us);

    g_encoder_count = encoder_now;
    g_measured_speed_mps = y_mps;
    g_control_force_n = uc_n;
    g_pwm_us = pwm_us;

    return true;
}

int main(void) {
    stdio_init_all();
    sleep_ms(1200);

    printf("State-space speed control at 50 Hz\r\n");

    pio_add_program(g_pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(g_pio, g_sm, PIN_AB, 0);

    pwm_esc_init(&g_esc, PIN_PWM_ESC, ESC_MIN_US, ESC_NEUTRAL_US, ESC_MAX_US);

    state_space_control_initialize();

    g_prev_encoder_count = quadrature_encoder_get_count(g_pio, g_sm);
    g_encoder_count = g_prev_encoder_count;

    if (!add_repeating_timer_ms(-CONTROL_PERIOD_MS, control_timer_callback, NULL, &g_control_timer)) {
        printf("ERROR: failed to start 20 ms control timer\r\n");
        while (true) {
            tight_loop_contents();
        }
    }

    printf("Using default reference speed: %.2f m/s\r\n", REF_SPEED_MPS_DEFAULT);

    if (DEBUG) {
        printf("DEBUG=true -> serial mode\r\n");
        printf("Printing: elapsed_ms, encoder_count, speed_mps, force_n, ref_speed, pwm_us, last_interval, callback_count, min_interval, max_interval\r\n");
    } else if (!micro_ros_init_subscription()) {
        printf("micro-ROS agent not reachable, continuing with serial status output\r\n");
    }

    while (true) {
        if (!DEBUG && g_micro_ros_connected) {
            (void)rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(100));
            continue;
        }

        int32_t encoder_count;
        float speed_mps;
        float force_n;
        float ref_speed;
        uint16_t pwm_us;
        uint32_t last_interval;
        uint32_t callback_count;
        uint32_t min_interval;
        uint32_t max_interval;
        uint32_t elapsed_ms;

        uint32_t irq_state = save_and_disable_interrupts();
        encoder_count = g_encoder_count;
        speed_mps = g_measured_speed_mps;
        force_n = g_control_force_n;
        ref_speed = g_ref_speed_mps;
        pwm_us = g_pwm_us;
        last_interval = g_last_interval_us;
        callback_count = g_callback_count;
        min_interval = g_min_interval_us;
        max_interval = g_max_interval_us;
        restore_interrupts(irq_state);

        elapsed_ms = to_ms_since_boot(get_absolute_time());

         printf("elapsed_ms=%lu encoder_count=%ld speed_mps=%.4f force_n=%.4f ref_speed=%.4f pwm_us=%u last_interval=%lu callback_count=%lu min_interval=%lu max_interval=%lu\r\n",
               (unsigned long)elapsed_ms,
               (long)encoder_count,
             (double)speed_mps,
             (double)force_n,
             (double)ref_speed,
             (unsigned int)pwm_us,
             (unsigned long)last_interval,
             (unsigned long)callback_count,
             (unsigned long)min_interval,
             (unsigned long)max_interval);
        sleep_ms(100);
    }
}
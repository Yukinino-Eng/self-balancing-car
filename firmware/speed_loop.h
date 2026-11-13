#ifndef SPEED_LOOP_H
#define SPEED_LOOP_H
#include <stdint.h>
#ifdef PD_HOLD_TEST
#define SPEED_DEFAULT_KP 1.5f
#define SPEED_DEFAULT_KI 0.5f
#define SPEED_DEFAULT_TILT_DEG 2.0f
#define SPEED_MAX_KP 20.0f
#define SPEED_MAX_KI 10.0f
#define SPEED_MAX_TILT_DEG 5.0f
#define SPEED_PERIOD_MS 20u
#define SPEED_FILTER_SECONDS 0.08f
#define SPEED_TARGET_SLEW_DPS 10.0f
/* Differential damping only: PWM percentage points per 1000 counts/s.
   No heading integral, and no motion from a zero common balance request. */
#define WHEEL_SYNC_KP 2.0f
#define WHEEL_SYNC_LIMIT_PERCENT 1.0f
/* Stronger wheel-speed matching only during the forward task and stopping. */
#define TASK_WHEEL_SYNC_KP 6.0f
#define TASK_WHEEL_SYNC_LIMIT_PERCENT 2.0f
/* Integral of signed encoder speed difference, expressed as percentage/CM.
   This is a differential wheel PI; the existing mean-speed PI stays intact. */
#define TASK_WHEEL_SYNC_POSITION_GAIN 0.5f
#define TASK_WHEEL_SYNC_INTEGRAL_LIMIT 1.5f
#define TASK_WHEEL_SYNC_FULL_SPEED_CMS 1.0f
#define WHEEL_SYNC_FILTER_SECONDS 0.04f
#define WHEEL_SYNC_SLEW_PPS 5.0f
/* Gain units: degrees per (1000 encoder counts/s), degrees per 1000 counts. */
extern volatile float speed_kp, speed_ki, speed_tilt_limit_deg;
extern volatile float speed_left_cps, speed_right_cps, speed_filtered_cps;
extern volatile float speed_integral_deg, speed_target_body_deg;
extern volatile float speed_error_cps;
extern volatile float speed_wheel_difference_cps, speed_wheel_correction_percent;
extern volatile float speed_wheel_integral_percent;
extern volatile float speed_wheel_motion_blend;
extern volatile uint32_t speed_update_count, speed_state;
void speed_loop_init(void);
void speed_loop_reset(void);
void speed_loop_poll(void);
uint32_t speed_loop_config_valid(float kp, float ki, float tilt);
uint32_t speed_loop_command_enabled(void);
#ifdef PARALLEL_PID_TRIAL
/* Encoder synchronization across balance, travel and settling. */
/* Encoder position outer loop; released during disturbances and tasks. */
#define POSITION_HOLD_WINDOW_CM 5.0f
#define POSITION_HOLD_FREE_CM 0.5f
#define POSITION_HOLD_KP 1.5f /* cm/s per cm outside the free band. */
#define POSITION_HOLD_MAX_CMS 3.0f
#define POSITION_HOLD_ENTER_DEG 4.0f
#define POSITION_HOLD_EXIT_DEG 5.0f
#define POSITION_HOLD_ENTER_DPS 25.0f
#define POSITION_HOLD_EXIT_DPS 35.0f
#define POSITION_HOLD_ENTER_CMS 6.0f
#define POSITION_HOLD_EXIT_CMS 8.0f
#define POSITION_HOLD_STABLE_MS 100u
enum { POSITION_HOLD_OFF=0,POSITION_HOLD_WAITING=1,POSITION_HOLD_ACTIVE=2,
       POSITION_HOLD_DISTURBED=3,POSITION_HOLD_TASK=4 };
extern const volatile uint32_t position_hold_api;
extern volatile uint32_t position_hold_state;
/* Capture the stop point; error is current minus target in chassis cm. */
void speed_loop_position_hold_at(float chassis_error_cm);
extern volatile float position_hold_offset_cm,position_hold_target_cms,speed_effective_reference_cps;
#define PAR_WHEEL_SYNC_KP 6.0f /* PWM pp / (1000 counts/s). */
#define PAR_WHEEL_SYNC_POSITION_GAIN 0.5f /* PWM pp / cm of wheel mismatch. */
#define PAR_WHEEL_SYNC_I_LIMIT 2.0f
#define PAR_WHEEL_SYNC_LIMIT 3.0f
#define PAR_WHEEL_SYNC_SLEW_PPS 20.0f
#define PAR_WHEEL_SYNC_I_TILT_DEG 8.0f
extern volatile float speed_parallel_kp,speed_parallel_ki,speed_parallel_limit_percent;
uint32_t speed_loop_parallel_config_valid(float kp,float ki,float limit);
extern volatile float speed_parallel_p_percent,speed_parallel_i_percent,speed_parallel_output_percent;
float speed_loop_parallel_apply(float angle_pwm,float forward_limit,float backward_limit);
#endif
#endif
#endif

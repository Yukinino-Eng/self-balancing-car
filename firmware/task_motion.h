#ifndef TASK_MOTION_H
#define TASK_MOTION_H
#include <stdint.h>
#include "wheel_odometry.h"
#ifdef PD_HOLD_TEST
#define TASK_DEFAULT_SPEED_CMS 3.0f
#define TASK_DEFAULT_RAMP_MS 1000u
#define TASK_DURATION_MS 5000u
#define TASK_STOP_RAMP_MS 300u
extern const volatile uint32_t task_stop_ramp_ms;
#define TASK_COUNTS_PER_CM WHEEL_COUNTS_PER_CM
#define TASK_DEFAULT_REVERSE_CM (-10.0f)
#define TASK_DISTANCE_KP 1.0f /* cm/s per cm of distance error. */
#define TASK_DISTANCE_TOLERANCE_CM 0.5f
#define TASK_DISTANCE_RELEASE_CM 0.8f
enum { TASK_IDLE=0,TASK_BALANCE=1,TASK_FORWARD=2,TASK_SETTLING=3,TASK_DONE=4,TASK_BRAKING=5,TASK_DISTANCE=6,TASK_APPROACH=7,TASK_FAILED=8 };
enum { TASK_MODE_TIMED=0,TASK_MODE_DISTANCE=1 };
enum { TASK_FINISH_NONE=0,TASK_FINISH_TIME=1,TASK_FINISH_DISTANCE=2,TASK_FINISH_EARLY_STOP=3,TASK_FINISH_TIMEOUT=4,TASK_FINISH_OBSTACLE=5,TASK_FINISH_SENSOR=6,TASK_FINISH_BOUND=7,TASK_FINISH_SETTLE=8 };
enum { TASK_RESULT_IDLE=0,TASK_ACCEPTED=1,TASK_BUSY=2,TASK_INVALID=3,TASK_STALE=4 };
extern const volatile uint32_t task_continuous,task_duration_ms;
extern volatile float task_speed_cms,task_reference_cps,task_distance_cm;
extern volatile float task_left_distance_cm,task_right_distance_cm,task_wheel_error_cm;
extern volatile float task_requested_distance_cm,task_target_distance_cm,task_distance_error_cm;
extern volatile uint32_t task_mode,task_finish_reason,task_distance_timeout_ms;
extern volatile uint32_t task_distance_holding;
extern volatile uint32_t task_ramp_ms,task_state,task_command,task_elapsed_ms,task_remaining_ms;
extern volatile uint32_t task_request_id,task_result_id,task_result,task_run_count,task_completed_count;
#ifdef LEARN_SIMULATOR
extern uint32_t task_sim_timed;
extern uint32_t task_sim_manual_distance;
#endif
uint32_t task_motion_config_valid(float speed,uint32_t ramp);
uint32_t task_motion_distance_valid(float distance);
uint32_t task_motion_is_continuous(void);
extern const volatile uint32_t task_mailbox_api;
extern volatile uint32_t task_panel_command,task_panel_request_id,task_panel_expected_epoch;
extern volatile uint32_t task_panel_result_id,task_panel_result,task_cancel_epoch;
void task_motion_init(void);
void task_motion_begin_session(void);
void task_motion_end_session(void);
void task_motion_request(uint32_t command);
void task_motion_poll(uint32_t now);
/* Task telemetry uses the new heading; this adapter returns the unchanged
 * chassis-frame reference for the existing speed controller. */
extern const volatile uint32_t task_direction_reversed;
float task_motion_control_reference(void);
#if defined(CAR_OLED_RANGE) || defined(PERIPHERAL_SELFTEST)
#define TASK_HAS_OBSTACLE 1
#define AVOID_TRIGGER_MM 180u
#define AVOID_HOLD_MS 5000u
#define AVOID_RANGE_STALE_MS 300u
#define AVOID_MAX_SPEED_CMS 3.0f
#define AVOID_MAX_RAMP_MS 1000u
#define AVOID_MAX_FORWARD_CM 100.0f
#define AVOID_APPROACH_TIMEOUT_MS 60000u
#define AVOID_SETTLE_TIMEOUT_MS 10000u
enum { AVOID_OFF=0,AVOID_APPROACH=1,AVOID_BRAKING=2,AVOID_HOLD=3,AVOID_REVERSE=4,AVOID_COMPLETE=5,AVOID_ABORTING=6,AVOID_FAILED=7,AVOID_CANCELLED=8 };
enum { AVOID_REASON_NONE=0,AVOID_REASON_SENSOR=1,AVOID_REASON_TIME=2,AVOID_REASON_DISTANCE=3,AVOID_REASON_REVERSE_TIMEOUT=4,AVOID_REASON_USER=5,AVOID_REASON_SETTLE=6,AVOID_REASON_CONFIG=7 };
extern volatile uint32_t avoid_state,avoid_reason,avoid_hold_remaining_ms,avoid_stop_range_mm;
extern volatile uint32_t avoid_run_count,avoid_completed_count;
extern volatile float avoid_reverse_cm;
uint32_t task_motion_avoid_active(void);
#endif
#endif
#endif

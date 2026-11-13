#ifndef HOLD_TUNING_H
#define HOLD_TUNING_H
#include <stdint.h>
#ifdef PD_HOLD_TEST
#define TUNE_MAX_WINDOW_MS 30000u
#define TUNE_MAX_STEP_MS 200u
#define TUNE_MAX_BOOST_COUNTS 40u
enum { TUNE_IDLE=0, TUNE_APPLIED=1, TUNE_BUSY=2, TUNE_INVALID=3 };
extern const volatile uint32_t tune_api_version;
extern volatile float tune_requested_kp, tune_requested_kd;
extern volatile float tune_requested_backward_kp;
extern volatile float tune_requested_speed_kp, tune_requested_speed_ki, tune_requested_speed_tilt;
extern volatile uint32_t tune_requested_backward_limit;
extern volatile uint32_t tune_requested_gyro_predict_ms;
extern volatile uint32_t tune_requested_limit, tune_requested_window_ms;
extern volatile uint32_t tune_requested_step_ms, tune_requested_boost_counts;
extern volatile uint32_t tune_requested_boost_base;
extern volatile uint32_t tune_requested_common_boost_counts;
extern volatile float tune_requested_task_speed;
extern volatile uint32_t tune_requested_task_ramp_ms;
extern volatile uint32_t tune_command, tune_status, tune_error_field;
extern volatile uint32_t tune_request_id, tune_result_id, tune_revision;
void hold_tuning_init(void);
void hold_tuning_poll(void);
uint32_t hold_tuning_active_valid(void);
void hold_tuning_export_record(uint32_t out[16]);
uint32_t hold_tuning_record_valid(const uint32_t data[16]);
uint32_t hold_tuning_load_record(const uint32_t data[16]);
#endif
#endif

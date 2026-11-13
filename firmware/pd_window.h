#ifndef PD_WINDOW_H
#define PD_WINDOW_H
#include <stdint.h>
#include "motor_guard.h"
#if defined(PD_HOLD_TEST)
#if !defined(PD_GROUND_TEST) || defined(PD_LOAD_TEST)
#error Hold_test_requires_ground_PD_without_fixed_load
#endif
#define PD_WINDOW_RUN_MS 6000u
#elif defined(PD_GROUND_TEST)
#define PD_WINDOW_RUN_MS 1000u
#else
#define PD_WINDOW_RUN_MS 150u
#endif
#if defined(PD_GROUND_TEST) && !defined(PD_WINDOW_TEST) && !defined(LEARN_SIMULATOR)
#error Ground_test_requires_PD_output_owner_or_simulator
#endif
#if defined(PD_LOAD_TEST) && !defined(PD_GROUND_TEST)
#error Load_test_requires_ground_policy
#endif
#ifdef PD_HOLD_TEST
#include "hold_config.h"
#define PD_WINDOW_LIMIT_PERCENT HOLD_OUTPUT_LIMIT_PERCENT
#define PD_HOLD_START_STEP_MS HOLD_START_STEP_MS
#define PD_HOLD_START_FULL_MS HOLD_START_FULL_MS
#else
#define PD_WINDOW_LIMIT_PERCENT 10
#endif
#define PD_WINDOW_REVERSE_OFF_MS 2u
enum { PD_IDLE=0, PD_RUNNING=1, PD_DONE=2, PD_REJECTED=3,
       PD_CANCELLED=4, PD_FAULT=5 };
enum { PD_FAULT_NONE=0, PD_FAULT_REQUEST=1, PD_FAULT_COMMAND=2,
       PD_FAULT_BUSY=3, PD_FAULT_START_NOT_FUSED=4, PD_FAULT_GUARD_BASE=10 };
/* 唯一命令入口：1一次固定窗口（原目标150ms，Ground目标1000ms）；8安全且冷却结束后清锁；9停止。
   其余变量只观察。限幅由编译配置决定，不由Watch参数延长时间。 */
/* Load targets additionally accept 2:+10, 3:-10, 4:+15, 5:-15 percent.
   Values are compile-time steps; mirror writes cannot select an output. */
extern const volatile uint32_t pd_load_test_enabled;
extern volatile int32_t pd_window_fixed_percent;
extern volatile uint32_t pd_window_command;
extern volatile uint32_t pd_window_status, pd_window_active;
extern volatile uint32_t pd_window_latched, pd_window_fault;
extern volatile uint32_t pd_window_remaining_ms, pd_window_run_count;
extern volatile int32_t pd_window_applied_percent;
extern volatile float pd_window_requested_percent;
extern volatile uint32_t pd_window_peak_percent, pd_window_reverse_count;
extern volatile uint32_t pd_window_reverse_wait;
extern volatile uint32_t pd_window_gyro_only_frames, pd_window_gyro_only_peak_age_ms;
#ifdef PD_HOLD_TEST
extern volatile uint32_t pd_window_right_boost_updates;
extern volatile int32_t pd_window_applied_counts;
extern volatile uint32_t hold_start_step_ms, hold_start_full_ms, hold_output_limit_percent;
extern volatile uint32_t hold_backward_limit_percent, hold_backward_start_full_ms;
#endif
extern const volatile uint32_t pd_window_connected;
#ifdef PD_HOLD_TEST
extern volatile uint32_t pd_window_limit_ms;
#else
extern const volatile uint32_t pd_window_limit_ms;
#endif
#ifdef LEARN_SIMULATOR
/* 模拟调试器恰在命令读出后写入，用于验证不丢新命令。 */
extern volatile uint32_t pd_window_sim_arrival;
#endif
/* 首个故障快照；清锁后保留，接受下一窗口或初始化才失效。 */
extern volatile uint32_t pd_fault_snapshot_valid, pd_fault_reason, pd_fault_ms;
extern volatile uint32_t pd_fault_elapsed_ms, pd_fault_sample_age_ms;
extern volatile uint32_t pd_fault_run_count, pd_fault_was_active;
extern volatile int32_t pd_fault_applied_percent;
extern volatile motor_guard_snapshot_t pd_fault_sample;
void pd_window_init(void);
void pd_window_cancel(void);
void pd_window_poll(void);
void pd_window_tick(void);
#endif

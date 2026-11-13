#include "pd_window.h"
#include "board.h"
#include "runtime.h"
#include "motor_guard.h"
#include "balance_preview.h"
#include "encoder.h"
#include "imu.h"
#include "hold_tuning.h"
#include "speed_loop.h"
#include "local_control.h"
#include "task_motion.h"

#ifdef PD_LOAD_TEST
const volatile uint32_t pd_load_test_enabled = 1u;
static int32_t fixed_percent;
#else
const volatile uint32_t pd_load_test_enabled = 0u;
#endif
volatile int32_t pd_window_fixed_percent = 0;
volatile uint32_t pd_window_command = 0u;
volatile uint32_t pd_window_status = PD_IDLE, pd_window_active = 0u;
volatile uint32_t pd_window_latched = 0u, pd_window_fault = PD_FAULT_NONE;
volatile uint32_t pd_window_remaining_ms = 0u, pd_window_run_count = 0u;
volatile int32_t pd_window_applied_percent = 0;
volatile float pd_window_requested_percent = 0.0f;
volatile uint32_t pd_window_peak_percent = 0u, pd_window_reverse_count = 0u;
volatile uint32_t pd_window_reverse_wait = 0u;
volatile uint32_t pd_window_gyro_only_frames = 0u, pd_window_gyro_only_peak_age_ms = 0u;
#ifdef PD_HOLD_TEST
volatile uint32_t pd_window_right_boost_updates = 0u;
volatile int32_t pd_window_applied_counts = 0;
volatile uint32_t hold_start_step_ms = PD_HOLD_START_STEP_MS;
volatile uint32_t hold_start_full_ms = PD_HOLD_START_FULL_MS;
volatile uint32_t hold_output_limit_percent = HOLD_DEFAULT_FORWARD_LIMIT_PERCENT;
volatile uint32_t hold_backward_limit_percent = HOLD_DEFAULT_BACKWARD_LIMIT_PERCENT;
volatile uint32_t hold_backward_start_full_ms = (HOLD_DEFAULT_BACKWARD_LIMIT_PERCENT-1u)*HOLD_START_STEP_MS;
#endif
#ifdef PD_WINDOW_TEST
const volatile uint32_t pd_window_connected = 1u;
#else
const volatile uint32_t pd_window_connected = 0u;
#endif
/* 只读观察值；计时策略使用编译常量，不由Watch改时长。 */
#ifdef PD_HOLD_TEST
volatile uint32_t pd_window_limit_ms = PD_WINDOW_RUN_MS;
static uint32_t run_limit_ms = PD_WINDOW_RUN_MS;
#else
const volatile uint32_t pd_window_limit_ms = PD_WINDOW_RUN_MS;
#endif
#ifdef LEARN_SIMULATOR
volatile uint32_t pd_window_sim_arrival = 0u;
#endif
volatile uint32_t pd_fault_snapshot_valid = 0u, pd_fault_reason = 0u, pd_fault_ms = 0u;
volatile uint32_t pd_fault_elapsed_ms = 0u, pd_fault_sample_age_ms = 0u;
volatile uint32_t pd_fault_run_count = 0u, pd_fault_was_active = 0u;
volatile int32_t pd_fault_applied_percent = 0;
volatile motor_guard_snapshot_t pd_fault_sample;
static volatile uint32_t start_ms, cooldown_ms, cooldown_active;
static uint32_t last_preview_update, last_off_ms;
static int32_t desired_units, last_direction, applied_units;

/* 仅在主循环临界区或SysTick中调用：本目标独占电机输出。 */

static void stop_window(uint32_t status)
{
    board_motor_off();
    if (pd_window_active != 0u) {
        encoder_sample();
        cooldown_ms = uptime_ms;
        cooldown_active = 1u;
    }
    pd_window_active = 0u;
#ifdef PD_HOLD_TEST
    speed_loop_reset();
    task_motion_end_session();
    local_control_cancel();
#endif
    pd_window_applied_percent = 0;
    applied_units = 0;
#ifdef PD_HOLD_TEST
    pd_window_applied_counts = 0;
#endif
    pd_window_requested_percent = 0.0f;
    pd_window_remaining_ms = 0u;
    pd_window_reverse_wait = 0u;
    desired_units = 0;
    pd_window_fixed_percent = 0;
#ifdef PD_LOAD_TEST
    fixed_percent = 0;
#endif
    pd_window_status = status;
}

void pd_window_cancel(void)
{
    uint32_t saved=__get_PRIMASK();
    __disable_irq();pd_window_command=0u;stop_window(PD_CANCELLED);__set_PRIMASK(saved);
}

static void fault_window(uint32_t reason)
{
    uint32_t was_active = pd_window_active;
    uint32_t fault_ms = uptime_ms;
    int32_t applied = pd_window_applied_percent;
#ifdef PD_HOLD_TEST
    pd_window_latched = 0u; /* No persistent fault lock in Hold. */
#else
    pd_window_latched = 1u;
#endif
    if (reason != PD_FAULT_NONE) { pd_window_fault = reason; }
    pd_window_command = 0u;
    stop_window(PD_FAULT);          /* 先关输出，再记录，诊断不延迟停机。 */
    if (pd_fault_snapshot_valid == 0u) {
        pd_fault_reason = reason;
        pd_fault_ms = fault_ms;
        pd_fault_elapsed_ms = was_active != 0u ? (uint32_t)(fault_ms - start_ms) : 0u;
        pd_fault_run_count = pd_window_run_count;
        pd_fault_was_active = was_active;
        pd_fault_applied_percent = applied;
        motor_guard_copy_snapshot(&pd_fault_sample);
        pd_fault_sample_age_ms = (uint32_t)(fault_ms - pd_fault_sample.sample_ms);
        pd_fault_snapshot_valid = 1u; /* 最后发布；数据恢复不覆盖首个故障。 */
    }
}

static uint32_t guard_fault(void)
{
    uint32_t reason;
    if (imu_freeze != 0u || imu_frozen != 0u) {
        return PD_FAULT_GUARD_BASE + GUARD_FROZEN;
    }
    reason = motor_guard_check(uptime_ms);
    return reason == GUARD_READY ? PD_FAULT_NONE : PD_FAULT_GUARD_BASE + reason;
}

static uint32_t input_fault(void)
{
    uint32_t reason = guard_fault();
    float request = balance_request_percent;
#ifdef PD_HOLD_TEST
    float limit = (float)hold_output_limit_percent;
    float backward_limit = (float)hold_backward_limit_percent;
    if (hold_tuning_active_valid() == 0u) { return PD_FAULT_REQUEST; }
#else
    float limit = (float)PD_WINDOW_LIMIT_PERCENT;
    float backward_limit = limit;
#endif
    if (reason != PD_FAULT_NONE) { return reason; }
    if (balance_status != BALANCE_PREVIEW_READY ||
        !(request >= -backward_limit &&
          request <= limit)) {
        return PD_FAULT_REQUEST;      /* 包括NaN/Inf与非法增益。 */
    }
    return PD_FAULT_NONE;
}

static void drive_desired(void)
{
    int32_t wanted=desired_units, sign;
    uint32_t duty;
#ifdef PD_HOLD_TEST
    uint32_t step=hold_start_step_ms;
    uint32_t limit=4u*(wanted<0 ? hold_backward_limit_percent : hold_output_limit_percent);
    uint32_t startup_cap=step==0u ? limit : 4u+4u*(uint32_t)(uptime_ms-start_ms)/step;
    uint32_t old_a=observed_pwm_a, old_b=observed_pwm_b;
    float correction=speed_wheel_correction_percent*4.0f;
    int32_t sync=correction>=0.0f ? (int32_t)(correction+0.5f) : -(int32_t)(-correction+0.5f);
    if (startup_cap<limit) {
        if (wanted>(int32_t)startup_cap) wanted=(int32_t)startup_cap;
        if (wanted<-(int32_t)startup_cap) wanted=-(int32_t)startup_cap;
    }
#endif
    sign=wanted>0 ? 1 : -1;
    if (wanted==0) {
        if (applied_units!=0) { board_motor_off(); applied_units=0; last_off_ms=uptime_ms; }
        pd_window_applied_percent=0;
#ifdef PD_HOLD_TEST
        pd_window_applied_counts=0;
#endif
        pd_window_reverse_wait=0u; return;
    }
    if (applied_units!=0 && sign!=last_direction) {
        board_motor_off(); applied_units=0; pd_window_applied_percent=0;
#ifdef PD_HOLD_TEST
        pd_window_applied_counts=0;
#endif
        last_off_ms=uptime_ms; pd_window_reverse_wait=1u; return;
    }
    if (applied_units==0 && last_direction!=0 && sign!=last_direction &&
        (uint32_t)(uptime_ms-last_off_ms)<PD_WINDOW_REVERSE_OFF_MS) {
        pd_window_reverse_wait=1u; return;
    }
    pd_window_reverse_wait=0u;
#ifndef PD_HOLD_TEST
    if (applied_units==wanted) return;
#endif
    duty=(uint32_t)(wanted>0 ? wanted : -wanted);
    if (last_direction!=0 && sign!=last_direction) ++pd_window_reverse_count;
#ifdef PD_LOAD_TEST
    if (fixed_percent!=0) board_load_start(sign>0 ? 5u : 6u,duty);
    else
#endif
    {
#ifdef PD_HOLD_TEST
        board_hold_start_counts(sign>0 ? 5u : 6u,duty,sync);
#else
        board_motor_start(sign>0 ? 5u : 6u,duty);
#endif
    }
    last_direction=sign; applied_units=wanted;
#ifdef PD_HOLD_TEST
    pd_window_applied_counts=wanted;
    /* Legacy whole-percent diagnostic; actual state uses native PWM counts. */
    pd_window_applied_percent=sign*(int32_t)((duty+2u)/4u);
    if ((old_a!=observed_pwm_a || old_b!=observed_pwm_b) && observed_pwm_b>observed_pwm_a)
        ++pd_window_right_boost_updates;
    duty=((observed_pwm_a>observed_pwm_b ? observed_pwm_a : observed_pwm_b)*100u+
          PWM_PERIOD_COUNTS-1u)/PWM_PERIOD_COUNTS;
#else
    pd_window_applied_percent=wanted;
#endif
    if (duty>pd_window_peak_percent) pd_window_peak_percent=duty;
}
void pd_window_init(void)
{
    const motor_guard_snapshot_t empty = {0};
    pd_fault_snapshot_valid = 0u; pd_fault_reason = 0u; pd_fault_ms = 0u;
    pd_fault_elapsed_ms = 0u; pd_fault_sample_age_ms = 0u;
    pd_fault_run_count = 0u; pd_fault_was_active = 0u;
    pd_fault_applied_percent = 0; pd_fault_sample = empty;
    pd_window_command = 0u;
    pd_window_active = 0u;
#ifdef PD_HOLD_TEST
    speed_loop_reset();
    task_motion_end_session();
    local_control_cancel();
#endif
    pd_window_latched = 0u;
    pd_window_fault = PD_FAULT_NONE;
    pd_window_run_count = 0u;
    pd_window_gyro_only_frames = 0u; pd_window_gyro_only_peak_age_ms = 0u;
#ifdef PD_HOLD_TEST
    pd_window_right_boost_updates = 0u;
    hold_tuning_init(); run_limit_ms = pd_window_limit_ms;
    task_motion_init();
#endif
    pd_window_peak_percent = 0u;
    pd_window_reverse_count = 0u;
    start_ms = 0u; cooldown_ms = 0u; cooldown_active = 0u;
    last_preview_update = 0u; last_off_ms = 0u; last_direction = 0;
    stop_window(PD_IDLE);
}

static void poll_locked(void)
{
    uint32_t command = pd_window_command;
    uint32_t reason;
    float request;
#ifdef LEARN_SIMULATOR
    if (pd_window_sim_arrival != 0u) {
        pd_window_command = pd_window_sim_arrival;
        pd_window_sim_arrival = 0u;
    }
#endif
    /* 读到0时不回写0：否则调试器刚写入的新命令可能被覆盖。
       单一发送方应等本次命令处理/状态反馈后再发下一条。 */
    if (command != 0u) { pd_window_command = 0u; }
    if (command == 9u) { stop_window(PD_CANCELLED); return; }
#ifdef PD_HOLD_TEST
    hold_tuning_poll();
#endif
    if (command == 8u) {
        reason = input_fault();
        if (pd_window_active != 0u) { fault_window(PD_FAULT_BUSY); }
        else if (reason != PD_FAULT_NONE) { fault_window(reason); }
        else if (cooldown_active != 0u && (uint32_t)(uptime_ms-cooldown_ms) < 1000u) {
            stop_window(PD_REJECTED);
        } else {
            pd_window_latched = 0u;
            pd_window_fault = PD_FAULT_NONE;
            stop_window(PD_IDLE);
        }
        return;                     /* 清锁绝不顺便启动。 */
    }
    if (command != 0u) {
#ifdef PD_LOAD_TEST
        if (command < 1u || command > 5u) { fault_window(PD_FAULT_COMMAND); return; }
#else
        if (command != 1u) { fault_window(PD_FAULT_COMMAND); return; }
#endif
        if (pd_window_active != 0u) { fault_window(PD_FAULT_BUSY); return; }
        reason = input_fault();
        if (pd_window_latched != 0u || reason != PD_FAULT_NONE) {
            fault_window(reason); return;
        }
#if defined(PD_GROUND_TEST) && !defined(PD_HOLD_TEST)
        if (imu_fusion_mode != IMU_FUSION_FUSED) {
            fault_window(PD_FAULT_START_NOT_FUSED); return;
        }
#endif
        if (cooldown_active != 0u && (uint32_t)(uptime_ms-cooldown_ms) < 1000u) {
            stop_window(PD_REJECTED); return;
        }
        pd_window_fault = PD_FAULT_NONE;
        pd_fault_snapshot_valid = 0u; /* 新窗口开始后才允许记录新的首个故障。 */
#ifdef PD_LOAD_TEST
        fixed_percent = command == 2u ? 10 : command == 3u ? -10 :
                        command == 4u ? 15 : command == 5u ? -15 : 0;
        pd_window_fixed_percent = fixed_percent;
#endif
        encoder_sample();
        board_watchdog_start();
        start_ms = uptime_ms;
#ifdef PD_HOLD_TEST
        run_limit_ms = pd_window_limit_ms;
        pd_window_remaining_ms = run_limit_ms;
#else
        pd_window_remaining_ms = PD_WINDOW_RUN_MS;
#endif
        pd_window_active = 1u;
#ifdef PD_HOLD_TEST
        task_motion_begin_session();
        speed_loop_reset();
#endif
        pd_window_status = PD_RUNNING;
        ++pd_window_run_count;
        pd_window_gyro_only_frames = 0u; pd_window_gyro_only_peak_age_ms = 0u;
#ifdef PD_HOLD_TEST
        pd_window_right_boost_updates = 0u;
#endif
        pd_window_peak_percent = 0u; pd_window_reverse_count = 0u;
        pd_window_reverse_wait = 0u;
        last_direction = 0; desired_units = 0;
        last_preview_update = balance_update_count - 1u; /* 首份建议立即应用。 */
    }
    if (pd_window_active == 0u) { return; }
    reason = input_fault();
    if (reason != PD_FAULT_NONE) { fault_window(reason); return; }
    if (last_preview_update != balance_update_count) {
        last_preview_update = balance_update_count;
        if (imu_fusion_mode == IMU_FUSION_GYRO_ONLY) {
            ++pd_window_gyro_only_frames;
            if (imu_gyro_only_age_ms > pd_window_gyro_only_peak_age_ms) {
                pd_window_gyro_only_peak_age_ms = imu_gyro_only_age_ms;
            }
        }
        request = balance_request_percent;
#ifdef PD_LOAD_TEST
        if (fixed_percent != 0) { request = (float)fixed_percent; }
        pd_window_fixed_percent = fixed_percent;
#endif
        pd_window_requested_percent = request;
        /* Quantize base request; the Hold board API applies the small-output trim. */
#ifdef PD_HOLD_TEST
        request*=4.0f; /* 0.25% command resolution, matching the hardware. */
#endif
        desired_units = request >= 0.0f ? (int32_t)(request + 0.5f) :
                                           -(int32_t)(-request + 0.5f);
    }
    drive_desired();
}

void pd_window_poll(void)
{
    /* 不包含I2C等待；使主循环与SysTick的停机操作不交错。 */
    __disable_irq();
    poll_locked();
    __enable_irq();
}

void pd_window_tick(void)
{
    uint32_t elapsed, reason;
#ifdef PD_HOLD_TEST
    uint32_t limit = run_limit_ms;
#else
    uint32_t limit = PD_WINDOW_RUN_MS;
#endif
    if (pd_window_active == 0u) { return; }
    reason = guard_fault();
    if (reason != PD_FAULT_NONE) { fault_window(reason); return; }
#ifdef PD_HOLD_TEST
    if(task_motion_is_continuous()){pd_window_remaining_ms=0u;return;}
#endif
    elapsed = (uint32_t)(uptime_ms - start_ms);
    if (elapsed >= limit) { stop_window(PD_DONE); }
    else { pd_window_remaining_ms = limit - elapsed; }
}

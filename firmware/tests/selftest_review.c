/* Regression for review findings; simulated IO, never hardware. */
#ifdef LEARN_SIMULATOR
#include "selftest.h"
#include "task_motion.h"
#include "speed_loop.h"
#include "pd_window.h"
#include "encoder.h"
#include "balance_preview.h"
#include "imu.h"
#include "local_control.h"
#include "peripherals.h"
extern volatile uint32_t selftest_failure_lines[32];
extern void peripherals_review_selftest_run(void);
static void check_at(uint32_t p,uint32_t line)
{
    ++selftest_checks;
    if(!p){if(selftest_failures<32u)selftest_failure_lines[selftest_failures]=100000u+line;++selftest_failures;}
}
#define check(p) check_at((p),__LINE__)
static void begin(void)
{
    board_motor_off();uptime_ms=0u;encoder_init();imu_init();balance_preview_init();pd_window_init();
    pd_window_active=1u;task_motion_begin_session();speed_loop_reset();
    imu_body_pitch_deg=0.0f;imu_pitch_rate_dps=0.0f;
    task_speed_cms=3.0f;task_ramp_ms=1000u;task_requested_distance_cm=-10.0f;
    tof_valid=1u;tof_range_mm=150u;tof_last_ms=0u;
}
static void oframe(uint32_t now)
{uptime_ms=now;tof_last_ms=now;task_motion_poll(now);}
void park_selftest_run(void)
{
    begin();task_motion_request(3u);task_motion_poll(0u);imu_body_pitch_deg=3.0f;
    oframe(1000u);oframe(9999u);
    check(avoid_state==AVOID_BRAKING && task_state==TASK_SETTLING);
    oframe(10000u);
    check(avoid_state==AVOID_FAILED && task_state==TASK_FAILED && task_reference_cps==0.0f && pd_window_active);
    oframe(20000u);
    check(avoid_state==AVOID_FAILED && task_state==TASK_FAILED && avoid_completed_count==0u);
    begin();task_motion_request(1u);task_motion_request(9u);
    check(task_command==9u);
    task_motion_request(2u);check(task_command==9u);
    task_motion_poll(uptime_ms);
    check(task_state==TASK_BALANCE && task_reference_cps==0.0f && task_run_count==0u);
    /* A failed task requires an explicit recovery press, with no travel. */
    begin();task_motion_request(3u);task_motion_poll(0u);imu_body_pitch_deg=3.0f;
    oframe(1000u);oframe(10000u);
    local_sim_pressed=0u;local_control_init();uptime_ms=10020u;local_control_tick();uptime_ms=10040u;local_control_tick();
    local_sim_pressed=1u;uptime_ms=10060u;local_control_tick();uptime_ms=10080u;local_control_tick();
    local_sim_pressed=0u;uptime_ms=10100u;local_control_tick();uptime_ms=10120u;local_control_tick();
    check(task_command==9u);oframe(10120u);
    check(task_state==TASK_BALANCE && task_reference_cps==0.0f && task_run_count==1u && pd_window_active);
    /* Repeated cancel does not restart the exit timeout. */
    begin();tof_range_mm=500u;task_motion_request(3u);task_motion_poll(0u);imu_body_pitch_deg=3.0f;
    oframe(1000u);task_motion_request(9u);oframe(1000u);
    oframe(2000u);task_motion_request(9u);oframe(10000u);oframe(11000u);
    check(task_state==TASK_FAILED && avoid_state==AVOID_CANCELLED && task_reference_cps==0.0f);
    /* A button cancel revokes already-staged web travel, and its ACK is
     * independent of the local request/result sequence. */
    begin();task_panel_request_id=7u;task_panel_expected_epoch=task_cancel_epoch;
    task_motion_request(9u);task_panel_command=1u;task_motion_poll(0u);
    check(task_panel_result_id==7u && task_panel_result==TASK_STALE && task_state==TASK_BALANCE);
    begin();task_motion_request(1u);task_panel_request_id=8u;task_panel_expected_epoch=task_cancel_epoch;
    task_panel_command=9u;task_motion_poll(0u);
    check(task_panel_result_id==8u && task_panel_result==TASK_ACCEPTED && task_state==TASK_BALANCE && task_run_count==0u);
    begin();task_motion_request(1u);task_panel_request_id=9u;task_panel_expected_epoch=task_cancel_epoch;
    task_panel_command=1u;task_motion_poll(0u);
    check(task_panel_result_id==9u && task_panel_result==TASK_BUSY && task_state==TASK_FORWARD && task_mode==TASK_MODE_TIMED);
    begin();task_panel_request_id=10u;task_panel_expected_epoch=task_cancel_epoch;
    task_panel_command=1u;task_motion_poll(0u);
    check(task_panel_result_id==10u && task_panel_result==TASK_ACCEPTED && task_state==TASK_FORWARD);
    begin();task_panel_request_id=11u;task_panel_expected_epoch=task_cancel_epoch;
    pd_window_cancel();pd_window_active=1u;task_motion_begin_session();task_panel_command=1u;task_motion_poll(0u);
    check(task_panel_result_id==11u && task_panel_result==TASK_STALE && task_state==TASK_BALANCE);
    pd_window_cancel();
    peripherals_review_selftest_run();
}
#endif

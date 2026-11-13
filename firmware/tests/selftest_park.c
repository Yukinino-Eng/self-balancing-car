/* Parking transition regression with simulated encoders, not physical motion. */
#ifdef LEARN_SIMULATOR
#include "selftest.h"
#include "task_motion.h"
#include "speed_loop.h"
#include "pd_window.h"
#include "encoder.h"
#include "balance_preview.h"
#include "imu.h"
#include "motor_guard.h"
#ifdef PERIPHERAL_SELFTEST
#include "peripherals.h"
#endif
extern volatile uint32_t selftest_failure_lines[32];
static void check_at(uint32_t p,uint32_t line)
{
    ++selftest_checks;
    if(!p){if(selftest_failures<32u)selftest_failure_lines[selftest_failures]=80000u+line;++selftest_failures;}
}
#define check(p) check_at((p),__LINE__)
static float absolute(float value){return value<0.0f ? -value : value;}
static void begin(uint32_t command)
{
    board_motor_off();uptime_ms=0u;encoder_init();imu_init();balance_preview_init();pd_window_init();
    pd_window_active=1u;task_motion_begin_session();task_sim_manual_distance=1u;speed_loop_reset();
    hold_right_boost_counts=0u;hold_common_boost_counts=0u;balance_output_limited=0u;
    imu_body_pitch_deg=0.0f;imu_pitch_rate_dps=0.0f;
#ifdef PARALLEL_PID_TRIAL
    balance_kp=5.5f;balance_backward_kp=5.5f;balance_kd=0.15f; /* Saved Park baseline. */
#endif
    task_motion_request(command);task_motion_poll(0u);board_hold_start_counts(5u,16u,0);
}
static void step(int32_t left,int32_t right)
{
    encoder_e1_count=encoder_signed32((uint32_t)encoder_e1_count+(uint32_t)left);
    encoder_e2_count=encoder_signed32((uint32_t)encoder_e2_count+(uint32_t)right);
    uptime_ms+=20u;speed_loop_poll();
}
#ifdef PARALLEL_PID_TRIAL
#include "param_store.h"
#include "parallel_trial_config.h"
#include "local_control.h"
static void save_params(void)
{
    ++param_store_request_id;param_store_command=1u;param_store_poll();
}
static void boot_params(void)
{
    balance_preview_init();pd_window_init();local_control_init();speed_loop_init();param_store_init();
}
static void parallel_sync_checks(void)
{
    uint32_t i;float before,last;
    begin(0u);for(i=0u;i<30u;i++)step(-20,20);
    check(speed_wheel_integral_percent==0.0f && speed_wheel_correction_percent==0.0f);
    begin(0u);for(i=0u;i<40u;i++)step(-18,20);
    check(task_state==TASK_BALANCE && speed_wheel_integral_percent>0.5f && speed_wheel_correction_percent>1.0f);
    before=speed_wheel_integral_percent;for(i=0u;i<40u;i++)step(-20,20);
    check(absolute(speed_wheel_integral_percent-before)<0.001f && absolute(speed_wheel_correction_percent-before)<0.001f);
    begin(0u);for(i=0u;i<40u;i++)step(-20,18);
    check(speed_wheel_integral_percent < -0.5f && speed_wheel_correction_percent < -1.0f);
    begin(0u);board_hold_start_counts(6u,16u,0);for(i=0u;i<40u;i++)step(20,-18);
    check(speed_wheel_integral_percent>0.5f && speed_wheel_correction_percent>1.0f);
    begin(0u);board_motor_off();for(i=0u;i<40u;i++)step(-18,20);
    check(speed_wheel_integral_percent==0.0f && !observed_stby);
    begin(0u);hold_actuator_limited=1u;for(i=0u;i<40u;i++)step(-18,20);
    check(speed_wheel_integral_percent==0.0f);
    begin(0u);for(i=0u;i<30u;i++)step(-18,20);
    before=speed_wheel_integral_percent;board_hold_start_counts(5u,1u,12);step(-18,20);
    check(hold_sync_saturation_error_counts>0 && absolute(speed_wheel_integral_percent-before)<0.001f);
    step(-20,18);check(speed_wheel_integral_percent<before);
    begin(0u);imu_body_pitch_deg=9.0f;for(i=0u;i<30u;i++)step(-18,20);
    check(speed_wheel_integral_percent==0.0f && speed_wheel_correction_percent>0.0f);
    begin(0u);last=0.0f;for(i=0u;i<300u;i++){
        step(-20,10);
        if(absolute(speed_wheel_integral_percent)>2.0001f || absolute(speed_wheel_correction_percent)>3.0001f ||
           absolute(speed_wheel_correction_percent-last)>0.4001f)break;
        last=speed_wheel_correction_percent;
    }
    check(i==300u && speed_wheel_correction_percent<0.0f);
    begin(0u);for(i=0u;i<40u;i++)step(-18,20);
    before=speed_wheel_integral_percent;last=speed_wheel_correction_percent;
    speed_parallel_i_percent=0.4f;task_state=TASK_DONE;step(-20,20);
    check(absolute(speed_wheel_integral_percent-before)<0.001f &&
          absolute(speed_wheel_correction_percent-last)<=0.4001f && speed_parallel_i_percent==0.4f);
    task_motion_request(1u);task_motion_poll(uptime_ms);step(-20,20);
    check(task_state==TASK_FORWARD && absolute(speed_wheel_integral_percent-before)<0.001f);
    pd_window_cancel();check(speed_wheel_integral_percent==0.0f && speed_wheel_correction_percent==0.0f && !observed_stby);
    begin(0u);encoder_e1_count=encoder_signed32(0x7ffffff5u);encoder_e2_count=encoder_signed32(0x7ffffff5u);
    speed_loop_reset();step(-18,20);check(speed_wheel_integral_percent>0.0f && speed_wheel_difference_cps>0.0f);
    board_hold_start_counts(5u,16u,8);check(observed_pwm_a==24u && observed_pwm_b==8u);
    board_hold_start_counts(6u,16u,8);check(observed_pwm_a==8u && observed_pwm_b==24u);
    pd_window_cancel();
}
static void position_ready(void)
{
    uint32_t i;for(i=0u;i<26u;i++)step(0,0);
}
static void position_hold_checks(void)
{
    uint32_t i;
    begin(0u);for(i=0u;i<4u;i++)step(0,0);
    check(position_hold_state==POSITION_HOLD_WAITING && position_hold_target_cms==0.0f);
    step(0,0);step(0,0);
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm==0.0f);
    for(i=0u;i<16u;i++)step(-2,2);
    check(position_hold_offset_cm>0.4f && position_hold_offset_cm<0.5f && position_hold_target_cms==0.0f);
    for(i=0u;i<20u;i++)step(-2,2);
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm>1.0f &&
          position_hold_target_cms < -0.7f && position_hold_target_cms > -0.9f &&
          speed_effective_reference_cps<0.0f && speed_error_cps>0.0f);
    for(i=0u;i<64u;i++)step(-2,2);
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm>2.8f && position_hold_target_cms== -3.0f);
    begin(0u);position_ready();for(i=0u;i<140u;i++)step(2,-2);
    check(position_hold_offset_cm < -3.9f && position_hold_target_cms==3.0f && speed_effective_reference_cps>0.0f);
    imu_body_pitch_deg=3.5f;imu_pitch_rate_dps=20.0f;step(0,0);
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_target_cms>0.0f);
    speed_parallel_i_percent=0.4f;speed_wheel_integral_percent=0.3f;imu_body_pitch_deg=5.1f;step(0,0);
    check(position_hold_state==POSITION_HOLD_DISTURBED && position_hold_target_cms==0.0f &&
          speed_parallel_i_percent==0.4f && speed_wheel_integral_percent==0.3f);
    imu_body_pitch_deg=4.1f;imu_pitch_rate_dps=0.0f;for(i=0u;i<30u;i++)step(0,0);
    check(position_hold_state==POSITION_HOLD_DISTURBED);
    imu_body_pitch_deg=0.0f;position_ready();
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm==0.0f && position_hold_target_cms==0.0f);
    imu_pitch_rate_dps= -36.0f;step(0,0);
    check(position_hold_state==POSITION_HOLD_DISTURBED && position_hold_target_cms==0.0f);
    begin(0u);imu_pitch_rate_dps=26.0f;for(i=0u;i<40u;i++)step(0,0);
    check(position_hold_state==POSITION_HOLD_WAITING);
    begin(0u);position_ready();step(-100,100);
    check(position_hold_state==POSITION_HOLD_DISTURBED && position_hold_target_cms==0.0f && speed_loop_command_enabled());
    begin(0u);for(i=0u;i<100u;i++)step(-4,4);
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_target_cms<0.0f);
    begin(0u);position_ready();for(i=0u;i<140u;i++)step(-2,2);
    task_motion_request(1u);task_motion_poll(uptime_ms);step(0,0);
    check(task_state==TASK_FORWARD && position_hold_state==POSITION_HOLD_TASK &&
          position_hold_target_cms==0.0f && speed_effective_reference_cps== -task_reference_cps);
    task_state=TASK_SETTLING;task_reference_cps=0.0f;for(i=0u;i<10u;i++)step(0,0);
    check(position_hold_state==POSITION_HOLD_ACTIVE);
    task_state=TASK_DONE;position_ready();
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm==0.0f);
    speed_parallel_limit_percent=0.0f;step(0,0);
    check(position_hold_state==POSITION_HOLD_OFF && position_hold_target_cms==0.0f && speed_effective_reference_cps==0.0f);
    speed_loop_reset();check(position_hold_state==POSITION_HOLD_OFF && position_hold_offset_cm==0.0f);
    begin(0u);encoder_e1_count=encoder_signed32(0x80000010u);encoder_e2_count=encoder_signed32(0x7ffffff0u);
    speed_loop_reset();position_ready();for(i=0u;i<140u;i++)step(-2,2);
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm>3.9f && position_hold_target_cms<0.0f);
    begin(0u);position_ready();for(i=0u;i<30u;i++)step(2,2);
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm==0.0f && position_hold_target_cms==0.0f);
    begin(0u);for(i=0u;i<4u;i++)step(0,0);imu_body_pitch_deg=4.1f;step(0,0);
    imu_body_pitch_deg=0.0f;for(i=0u;i<4u;i++)step(0,0);
    check(position_hold_state==POSITION_HOLD_WAITING);
    step(0,0);step(0,0);check(position_hold_state==POSITION_HOLD_ACTIVE);
    imu_body_pitch_deg= -5.1f;step(0,0);
    check(position_hold_state==POSITION_HOLD_DISTURBED && position_hold_target_cms==0.0f);
    pd_window_cancel();check(position_hold_state==POSITION_HOLD_OFF && position_hold_target_cms==0.0f);
}
static void storage_checks(void)
{
    uint32_t i;
    pd_window_cancel();param_store_sim_clear();boot_params();
    balance_kp=6.0f;save_params();
    check(param_store_status==STORE_SAVED && param_store_sequence==1u);
    /* Turn the first save into an original v3 record by erasing only its extension. */
    for(i=PARAM_WORDS+5u;i<256u;i++)param_store_sim_pages[0][i]=0xFFFFFFFFu;
    boot_params();
    check(param_store_valid && param_store_sequence==1u && balance_kp==6.0f &&
          speed_parallel_kp==PAR_TRIAL_SPEED_KP && speed_parallel_ki==PAR_TRIAL_SPEED_KI &&
          speed_parallel_limit_percent==PAR_TRIAL_SPEED_LIMIT && param_store_dirty);
    speed_parallel_kp=25.0f;speed_parallel_ki=2.0f;speed_parallel_limit_percent=10.0f;save_params();
    check(param_store_status==STORE_SAVED && param_store_sequence==2u && !param_store_dirty);
    boot_params();
    check(speed_parallel_kp==25.0f && speed_parallel_ki==2.0f && speed_parallel_limit_percent==10.0f &&
          balance_kp==6.0f && !param_store_dirty && !pd_window_active && !observed_stby);
    param_store_sim_fail_after=0;save_params();
    check(param_store_status==STORE_SAVED && param_store_sequence==2u);
    speed_parallel_ki=3.0f;param_store_sim_fail_after=51;save_params();
    check(param_store_status==STORE_IO_ERROR && param_store_sequence==2u);
    param_store_sim_fail_after=-1;boot_params();
    check(speed_parallel_ki==2.0f && param_store_sequence==2u);
    param_store_sim_pages[1][PARAM_WORDS+6u]^=1u;
    boot_params();
    check(param_store_valid && balance_kp==6.0f && speed_parallel_kp==PAR_TRIAL_SPEED_KP &&
          speed_parallel_ki==PAR_TRIAL_SPEED_KI && param_store_dirty);
    speed_parallel_ki=51.0f;save_params();
    check(param_store_status==STORE_INVALID && param_store_sequence==2u);
    speed_parallel_ki=5.5f;speed_parallel_limit_percent=100.0f;
    hold_output_limit_percent=100u;hold_backward_limit_percent=100u;save_params();boot_params();
    check(param_store_valid && speed_parallel_limit_percent==100.0f && hold_output_limit_percent==100u &&
          hold_backward_limit_percent==100u && !param_store_dirty);
    param_store_sim_clear();boot_params();
}
static void parallel_checks(void)
{
    uint32_t i;
    float saved_i;
    check(speed_loop_parallel_config_valid(27.5f,5.5f,100.0f) &&
          !speed_loop_parallel_config_valid(27.5f,5.5f,100.01f));
    begin(0u);speed_parallel_limit_percent=100.0f;speed_parallel_ki=0.0f;
    hold_output_limit_percent=25u;hold_backward_limit_percent=30u;
    step(-100,100);balance_preview_evaluate(0.0f,0.0f,1u);
    check(absolute(speed_parallel_output_percent-27.5f)<0.001f && balance_request_percent==25.0f);
    speed_filtered_cps=0.0f;step(100,-100);balance_preview_evaluate(0.0f,0.0f,1u);
    check(absolute(speed_parallel_output_percent+27.5f)<0.001f && absolute(balance_request_percent+27.5f)<0.001f);
    begin(0u);speed_parallel_limit_percent=100.0f;speed_parallel_ki=0.0f;
    hold_output_limit_percent=25u;hold_backward_limit_percent=30u;
    for(i=0u;i<100u;i++){step(-100,100);balance_preview_evaluate(0.0f,0.0f,1u);}
    check(speed_parallel_output_percent==100.0f && balance_request_percent==25.0f &&
          balance_output_limited && speed_parallel_i_percent==0.0f);
    hold_output_limit_percent=100u;hold_backward_limit_percent=100u;
    board_hold_start_counts(5u,400u,0);
    check(observed_pwm_a==400u && observed_pwm_b==400u && observed_stby);
    board_hold_start_counts(6u,400u,0);
    check(observed_pwm_a==400u && observed_pwm_b==400u && observed_stby);
    begin(0u);step(-20,20);balance_preview_evaluate(0.0f,0.0f,1u);
    check(absolute(speed_parallel_p_percent-5.5f)<0.001f && absolute(speed_parallel_i_percent-0.022f)<0.001f);
    check(speed_target_body_deg==0.0f && balance_p_percent==0.0f && absolute(balance_request_percent-5.522f)<0.001f);
    saved_i=speed_parallel_i_percent;balance_preview_evaluate(BALANCE_DEG_TO_RAD,0.0f,1u);
    check(absolute(balance_request_percent-11.022f)<0.001f && speed_parallel_i_percent==saved_i);
    begin(0u);step(20,-20);balance_preview_evaluate(0.0f,0.0f,1u);
    check(absolute(balance_request_percent+5.522f)<0.001f);
    begin(1u);uptime_ms=1000u;task_motion_poll(uptime_ms);speed_loop_reset();step(0,0);
    balance_preview_evaluate(0.0f,0.0f,1u);
    check(task_reference_cps>0.0f && speed_error_cps>0.0f && balance_request_percent>0.0f);
    begin(2u);for(i=0u;i<50u;i++){step(0,0);balance_preview_evaluate(0.0f,0.0f,1u);}
    check(task_reference_cps<0.0f && speed_error_cps<0.0f && balance_request_percent<0.0f);
    begin(0u);hold_output_limit_percent=25u;hold_backward_limit_percent=30u;
    step(-20,20);balance_preview_evaluate(4.4f*BALANCE_DEG_TO_RAD,0.0f,1u);
    check(balance_request_percent==25.0f && balance_output_limited && speed_parallel_i_percent==0.0f);
    speed_parallel_i_percent=1.0f;speed_filtered_cps=0.0f;
    step(20,-20);balance_preview_evaluate(4.4f*BALANCE_DEG_TO_RAD,0.0f,1u);
    check(speed_parallel_i_percent<1.0f && speed_parallel_i_percent>0.97f);
    step(20,-20);balance_preview_evaluate(-5.4f*BALANCE_DEG_TO_RAD,0.0f,1u);
    check(balance_request_percent== -30.0f && balance_output_limited && speed_parallel_i_percent>0.97f);
    begin(0u);hold_actuator_limited=1u;pd_window_applied_counts=99;
    step(-20,20);balance_preview_evaluate(0.0f,0.0f,1u);
    check(speed_parallel_i_percent==0.0f);
    begin(0u);for(i=0u;i<25u;i++){step(-100,100);balance_preview_evaluate(0.0f,0.0f,1u);}
    check(speed_parallel_output_percent==13.75f && speed_parallel_i_percent==0.0f);
    balance_preview_evaluate(34.9f*BALANCE_DEG_TO_RAD,0.0f,1u);
    check(balance_status==BALANCE_PREVIEW_READY);
    balance_preview_evaluate(-34.9f*BALANCE_DEG_TO_RAD,0.0f,1u);
    check(balance_status==BALANCE_PREVIEW_READY);
    balance_preview_evaluate(-35.1f*BALANCE_DEG_TO_RAD,0.0f,1u);
    check(balance_status==BALANCE_TILT_LIMIT && balance_request_percent==0.0f);
    motor_guard_update(34.9f,1u,1u,0u,2u,uptime_ms);check(motor_guard_check(uptime_ms)==GUARD_READY);
    motor_guard_update(-34.9f,1u,1u,0u,2u,uptime_ms);check(motor_guard_check(uptime_ms)==GUARD_READY);
    motor_guard_update(35.1f,1u,1u,0u,2u,uptime_ms);check(motor_guard_check(uptime_ms)==GUARD_TILT);
    motor_guard_update(-35.1f,1u,1u,0u,2u,uptime_ms);check(motor_guard_check(uptime_ms)==GUARD_TILT);
    balance_preview_evaluate(35.1f*BALANCE_DEG_TO_RAD,0.0f,1u);
    check(balance_status==BALANCE_TILT_LIMIT && balance_request_percent==0.0f);
    pd_window_cancel();
    check(speed_parallel_output_percent==0.0f && speed_parallel_i_percent==0.0f && observed_stby==0u);
    begin(0u);speed_kp=0.0f;speed_ki=0.0f;speed_tilt_limit_deg=0.0f;
    step(-20,20);balance_preview_evaluate(0.0f,0.0f,1u);
    check(speed_loop_command_enabled() && balance_request_percent>0.0f);
    speed_parallel_limit_percent=0.0f;balance_preview_evaluate(0.0f,0.0f,1u);
    check(!speed_loop_command_enabled() && balance_request_percent==0.0f && speed_parallel_i_percent==0.0f);
}
#endif
static void position(float cm,uint32_t now)
{
    float counts=cm*TASK_COUNTS_PER_CM;
    int32_t right=counts>=0.0f ? (int32_t)(counts+0.5f) : -(int32_t)(-counts+0.5f);
    encoder_e1_count=right;encoder_e2_count=-right;uptime_ms=now;task_motion_poll(now);
}
#ifdef TASK_HAS_OBSTACLE
static void oframe(uint32_t now,uint32_t mm,float cm)
{
    tof_valid=mm!=0u;
    if(mm){tof_range_mm=mm;tof_last_ms=now;}
    position(cm,now);
}
static void obegin(uint32_t mm)
{
    begin(0u);task_speed_cms=3.0f;task_ramp_ms=1000u;task_requested_distance_cm=-10.0f;
    tof_valid=mm!=0u;tof_range_mm=mm;tof_last_ms=0u;
    task_motion_request(3u);task_motion_poll(0u);
}
static void to_hold(void)
{obegin(150u);oframe(1000u,150u,0.0f);oframe(1500u,150u,0.0f);}
static void button_click(uint32_t now)
{
    local_sim_pressed=1u;uptime_ms=now;local_control_tick();uptime_ms=now+20u;local_control_tick();
    local_sim_pressed=0u;uptime_ms=now+40u;local_control_tick();uptime_ms=now+60u;local_control_tick();
}
static void obstacle_checks(void)
{
    uint32_t timeout;
    obegin(500u);check(task_result==TASK_ACCEPTED && task_state==TASK_APPROACH && avoid_state==AVOID_APPROACH && task_reference_cps==0.0f);
    oframe(1000u,500u,3.0f);check(absolute(task_reference_cps-3.0f*TASK_COUNTS_PER_CM)<0.01f);
    oframe(6000u,500u,15.0f);check(task_state==TASK_APPROACH && task_reference_cps>0.0f);
    oframe(6100u,180u,15.3f);check(avoid_state==AVOID_BRAKING && task_state==TASK_BRAKING && avoid_stop_range_mm==180u);
    oframe(6250u,170u,15.6f);check(absolute(task_reference_cps-1.5f*TASK_COUNTS_PER_CM)<0.01f && pd_window_active);
    oframe(6400u,165u,15.8f);check(task_state==TASK_SETTLING && task_reference_cps==0.0f);
    oframe(6899u,165u,15.8f);check(avoid_state==AVOID_BRAKING);
    oframe(6900u,165u,15.8f);check(avoid_state==AVOID_HOLD && avoid_hold_remaining_ms==5000u);
    oframe(11899u,165u,15.8f);check(avoid_state==AVOID_HOLD && avoid_hold_remaining_ms==1u);
    oframe(11900u,165u,15.8f);check(avoid_state==AVOID_REVERSE && task_state==TASK_DISTANCE && task_target_distance_cm== -10.0f && task_reference_cps==0.0f);
    oframe(12100u,260u,5.8f);oframe(12600u,260u,5.8f);
    check(avoid_state==AVOID_COMPLETE && avoid_completed_count==1u && pd_window_active && task_reference_cps==0.0f);
    to_hold();check(avoid_state==AVOID_HOLD && avoid_stop_range_mm==150u); /* No forward pulse when already close. */
    tof_last_ms=6500u;uptime_ms=6500u;task_motion_request(9u);task_motion_poll(6500u);
    check(avoid_state==AVOID_CANCELLED && task_state==TASK_DONE && task_target_distance_cm==0.0f);
    oframe(9000u,150u,0.0f);check(avoid_state==AVOID_CANCELLED && task_reference_cps==0.0f);
    obegin(500u);oframe(1000u,500u,3.0f);oframe(1200u,0u,3.2f);
    check(avoid_state==AVOID_APPROACH); /* A single invalid frame does not restart the mission. */
    oframe(1301u,0u,3.3f);check(avoid_state==AVOID_ABORTING && avoid_reason==AVOID_REASON_SENSOR && pd_window_active);
    oframe(2301u,0u,4.0f);oframe(2801u,0u,4.0f);
    check(avoid_state==AVOID_FAILED && task_reference_cps==0.0f);
    oframe(4000u,500u,4.0f);check(avoid_state==AVOID_FAILED && task_state==TASK_DONE);
    obegin(0u);check(task_result==TASK_INVALID && task_state==TASK_BALANCE && !task_motion_avoid_active());
    obegin(500u);oframe(60000u,500u,10.0f);check(avoid_reason==AVOID_REASON_TIME && avoid_state==AVOID_ABORTING);
    obegin(500u);oframe(1000u,500u,100.0f);check(avoid_reason==AVOID_REASON_DISTANCE && avoid_state==AVOID_ABORTING);
    to_hold();oframe(6500u,150u,0.0f);timeout=task_distance_timeout_ms;
    oframe(6500u+timeout,150u,0.0f);check(avoid_state==AVOID_ABORTING && avoid_reason==AVOID_REASON_REVERSE_TIMEOUT && pd_window_active);
    to_hold();imu_body_pitch_deg=5.0f;oframe(2000u,150u,0.0f);
    check(avoid_state==AVOID_HOLD && avoid_hold_remaining_ms==5000u);
    imu_body_pitch_deg=0.0f;oframe(2100u,150u,0.0f);oframe(7000u,150u,0.0f);
    check(avoid_state==AVOID_HOLD && avoid_hold_remaining_ms==100u);
    oframe(7100u,150u,0.0f);check(avoid_state==AVOID_REVERSE);
    to_hold();oframe(2000u,150u,0.0f);task_motion_request(1u);task_motion_poll(2000u);
    check(task_result==TASK_BUSY && avoid_state==AVOID_HOLD);
    pd_window_cancel();check(avoid_state==AVOID_OFF && !task_command && !observed_stby);
    pd_window_active=1u;task_motion_begin_session();task_sim_manual_distance=1u;oframe(10000u,150u,0.0f);
    check(avoid_state==AVOID_OFF && task_state==TASK_BALANCE && task_reference_cps==0.0f);
    obegin(500u);task_motion_request(9u);task_motion_poll(0u);oframe(1000u,500u,0.0f);oframe(1500u,500u,0.0f);
    task_speed_cms=3.01f;task_motion_request(3u);task_motion_poll(1500u);check(task_result==TASK_INVALID);
    task_speed_cms=3.0f;task_ramp_ms=1001u;task_motion_request(3u);task_motion_poll(1500u);check(task_result==TASK_INVALID);
    task_ramp_ms=1000u;task_requested_distance_cm=10.0f;task_motion_request(3u);task_motion_poll(1500u);check(task_result==TASK_INVALID);
    pd_window_cancel();uptime_ms=0u;local_sim_pressed=1u;local_control_init();check(!local_task_mode && !pd_window_active);
    uptime_ms=20u;local_control_tick();check(local_task_mode==0u && !pd_window_active && !local_command);
    local_sim_pressed=0u;uptime_ms=40u;local_control_tick();uptime_ms=60u;local_control_tick();
    check(!pd_window_active && !task_command); /* Releasing a held boot key never starts. */
    local_task_mode=1u;pd_window_active=1u;task_motion_begin_session();task_sim_manual_distance=1u;task_speed_cms=3.0f;task_ramp_ms=1000u;task_requested_distance_cm=-10.0f;
    button_click(80u);check(task_command==3u);
    tof_valid=1u;tof_range_mm=500u;tof_last_ms=uptime_ms;task_motion_poll(uptime_ms);
    check(avoid_state==AVOID_APPROACH);
    button_click(200u);check(task_command==9u);
    task_motion_poll(uptime_ms);check(avoid_state==AVOID_ABORTING && avoid_reason==AVOID_REASON_USER);
    pd_window_cancel();local_sim_pressed=0u;local_control_init();check(local_task_mode==0u);
}
#endif
void park_selftest_run(void)
{
#ifndef PARALLEL_PID_TRIAL
    uint32_t i;
    float before,last;
#endif
#ifdef PARALLEL_PID_TRIAL
    parallel_sync_checks();
#else
    begin(1u);uptime_ms=1000u;task_motion_poll(uptime_ms);speed_loop_reset();
    for(i=0u;i<40u;i++)step(0,1);
    check(speed_wheel_motion_blend==1.0f && speed_wheel_integral_percent>0.25f && speed_wheel_correction_percent>0.5f);
#ifdef PARALLEL_PID_TRIAL
    speed_parallel_ki=0.0f;speed_parallel_i_percent=0.4f;
#else
    speed_ki=0.0f;speed_integral_deg=0.4f;
#endif
    task_motion_request(9u);task_motion_poll(uptime_ms);last=speed_wheel_correction_percent;
    for(i=0u;i<40u;i++){
        step(0,0);
        if(absolute(speed_wheel_correction_percent-last)>0.1001f)break;
        last=speed_wheel_correction_percent;
    }
    check(i==40u && speed_wheel_motion_blend>0.0f && speed_wheel_motion_blend<1.0f);
    for(i=0u;i<10u;i++)step(0,0);
    check(task_state==TASK_SETTLING && speed_wheel_motion_blend==0.0f && speed_wheel_integral_percent==0.0f);
#ifdef PARALLEL_PID_TRIAL
    balance_preview_evaluate(0.0f,0.0f,1u);
    check(speed_parallel_i_percent==0.4f);
#else
    check(speed_integral_deg==0.4f); /* Keep the existing common-speed trim; no blanket reset. */
#endif
    for(i=0u;i<30u;i++)step(0,1);
    check(task_state==TASK_DONE && speed_wheel_motion_blend==0.0f && speed_wheel_integral_percent==0.0f && absolute(speed_wheel_correction_percent-0.1f)<0.001f);
    /* Even when settling cannot finish, it must not keep full travel PI. */
    begin(1u);uptime_ms=1000u;task_motion_poll(uptime_ms);speed_loop_reset();
    for(i=0u;i<30u;i++)step(0,20);
    before=speed_wheel_correction_percent;task_state=TASK_SETTLING;step(0,20);
    check(task_state==TASK_SETTLING && speed_wheel_motion_blend==0.0f && speed_wheel_integral_percent==0.0f);
    check(speed_wheel_correction_percent>=before-0.1001f);
    for(i=0u;i<30u;i++)step(0,20);
    check(task_state==TASK_SETTLING && speed_wheel_correction_percent==1.0f);
#endif
    begin(2u);position(-9.55f,20u);
    check(task_distance_holding==1u && task_reference_cps==0.0f);
    position(-9.35f,40u);
    check(task_distance_holding==1u && task_reference_cps==0.0f && task_distance_error_cm < -0.6f);
    position(-10.65f,60u);
    check(task_distance_holding==1u && task_reference_cps==0.0f && task_distance_error_cm > 0.6f);
    position(-9.0f,80u);
    check(task_distance_holding==0u && task_reference_cps<0.0f && task_state==TASK_DISTANCE);
    position(-10.0f,100u);position(-10.0f,599u);
    check(task_state==TASK_DISTANCE);
    position(-10.0f,600u);
    check(task_state==TASK_DONE && task_finish_reason==TASK_FINISH_DISTANCE && pd_window_active==1u);
    begin(2u);position(-9.55f,20u);position(-9.35f,520u);
    check(task_state==TASK_DONE && task_finish_reason==TASK_FINISH_DISTANCE && absolute(task_distance_error_cm)<TASK_DISTANCE_RELEASE_CM);
    task_motion_request(2u);task_motion_poll(uptime_ms);
    check(task_state==TASK_DISTANCE && task_distance_holding==0u && task_reference_cps==0.0f);
    begin(1u);task_state=TASK_DONE;task_requested_distance_cm=10.0f;task_motion_request(2u);task_motion_poll(uptime_ms);
    position(9.55f,20u);position(9.35f,40u);
    check(task_distance_holding==1u && task_reference_cps==0.0f);
    position(9.0f,60u);
    check(task_distance_holding==0u && task_reference_cps>0.0f);
    pd_window_cancel();
    check(task_distance_holding==0u && speed_wheel_motion_blend==0.0f && observed_stby==0u);
#ifdef PARALLEL_PID_TRIAL
    parallel_checks();
    position_hold_checks();
    storage_checks();
#ifdef PERIPHERAL_SELFTEST
    obstacle_checks();
    peripherals_selftest_run();
#endif
#endif
}
#endif

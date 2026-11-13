/* Distance scheduler and button policy with fake encoders/IMU/PWM only. */
#ifdef LEARN_SIMULATOR
#include "selftest.h"
#include "task_motion.h"
#include "speed_loop.h"
#include "pd_window.h"
#include "local_control.h"
#include "encoder.h"
#include "imu.h"
#include "balance_preview.h"
extern volatile uint32_t selftest_failure_lines[32];
static void check_at(uint32_t p,uint32_t line)
{
    ++selftest_checks;
    if(!p){if(selftest_failures<32u)selftest_failure_lines[selftest_failures]=70000u+line;++selftest_failures;}
}
#define check(p) check_at((p),__LINE__)
static float absolute(float value){return value<0.0f ? -value : value;}
static void fresh(void)
{
    imu_valid=1u;imu_fusion_valid=1u;imu_fusion_mode=IMU_FUSION_FUSED;
    imu_freeze=0u;imu_frozen=0u;imu_seq+=2u;++imu_sample_count;
    imu_last_sample_ms=uptime_ms;imu_body_pitch_deg=0.0f;imu_pitch_rate_dps=0.0f;
    balance_preview_poll();motor_guard_poll();
}
static void begin(uint32_t now)
{
    board_motor_off();uptime_ms=0u;encoder_init();selftest_reset_guard();imu_init();
    balance_preview_init();pd_window_init();local_sim_pressed=0u;local_control_init();
    task_sim_manual_distance=1u;uptime_ms=now;fresh();pd_window_command=1u;pd_window_poll();
}
static void poll(uint32_t now){uptime_ms=now;task_motion_poll(now);}
static void request(uint32_t command){task_motion_request(command);task_motion_poll(uptime_ms);}
static void tick(uint32_t n)
{
    while(n--){++uptime_ms;if(uptime_ms%10u==0u)fresh();local_control_tick();pd_window_tick();pd_window_poll();local_control_poll();}
}
static void press(void){local_sim_pressed=1u;tick(30u);local_sim_pressed=0u;tick(30u);}
static void position(float cm)
{
    float counts=cm*TASK_COUNTS_PER_CM;
    int32_t right=counts>=0.0f ? (int32_t)(counts+0.5f) : -(int32_t)(-counts+0.5f);
    encoder_e1_count=right;encoder_e2_count=-right;
}
/* Ideal reference-following arithmetic, not a physical balance-car model. */
static uint32_t follow(float goal,uint32_t now)
{
    uint32_t i;
    int32_t whole;
    float fraction=0.0f,last=0.0f;
    begin(now);task_requested_distance_cm=goal;request(2u);
    for(i=0u;i<3000u && task_state!=TASK_DONE;i++){
        fraction+=task_reference_cps*0.02f;
        whole=(int32_t)fraction;fraction-=(float)whole;
        encoder_e1_count=encoder_signed32((uint32_t)encoder_e1_count+(uint32_t)whole);
        encoder_e2_count=encoder_signed32((uint32_t)encoder_e2_count-(uint32_t)whole);
        speed_filtered_cps=task_reference_cps;poll(uptime_ms+20u);
        if(absolute(task_reference_cps)>211.081f)return 0u;
        if(absolute(task_reference_cps-last)>4.223f &&
           !(task_distance_holding && task_reference_cps==0.0f && absolute(task_distance_error_cm)<=TASK_DISTANCE_TOLERANCE_CM))return 0u;
        last=task_reference_cps;
    }
    return task_state==TASK_DONE && task_finish_reason==TASK_FINISH_DISTANCE &&
           absolute(task_distance_cm-goal)<=0.51f && task_completed_count==1u && pd_window_active==1u;
}
void distance_selftest_run(void)
{
    uint32_t i,limit,runs;
    float v;
    union{uint32_t bits;float value;} nan;
    begin(100u);request(2u);
    check(task_result==TASK_ACCEPTED && task_mode==TASK_MODE_DISTANCE && task_state==TASK_DISTANCE && task_target_distance_cm==-10.0f && task_reference_cps==0.0f);
    poll(120u);check(task_reference_cps<0.0f && task_reference_cps > -4.223f);
    for(i=0u;i<50u;i++)poll(uptime_ms+20u);
    check(task_reference_cps < -211.0f && task_reference_cps > -211.1f);
    v=task_reference_cps;task_requested_distance_cm=-20.0f;task_speed_cms=10.0f;task_ramp_ms=2000u;poll(uptime_ms+20u);
    check(task_target_distance_cm==-10.0f && task_reference_cps==v);
    runs=task_run_count;request(1u);
    check(task_result==TASK_BUSY && task_run_count==runs && task_state==TASK_DISTANCE);
    position(-9.0f);poll(uptime_ms+20u);
    check(task_distance_error_cm > -1.01f && task_distance_error_cm < -0.99f && task_reference_cps>v);
    position(-10.7f);for(i=0u;i<80u;i++)poll(uptime_ms+20u);
    check(task_distance_error_cm>0.69f && task_reference_cps>0.0f && task_state==TASK_DISTANCE); /* Correct an overshoot in the opposite direction. */
    check(follow(-10.0f,0u));
    check(follow(10.0f,0u));
    check(follow(-10.0f,0xFFFFFF00u));
    begin(0u);request(2u);position(-10.0f);speed_filtered_cps=-200.0f;poll(20u);poll(700u);
    check(task_state==TASK_DISTANCE && task_completed_count==0u); /* Distance alone is insufficient. */
    speed_filtered_cps=0.0f;poll(720u);poll(1219u);
    check(task_state==TASK_DISTANCE);
    position(-9.0f);poll(1220u);position(-10.0f);poll(1240u);poll(1739u);
    check(task_state==TASK_DISTANCE); /* Leaving the band resets continuous settling. */
    poll(1740u);
    check(task_state==TASK_DONE && task_finish_reason==TASK_FINISH_DISTANCE && task_completed_count==1u && pd_window_active==1u);
    poll(2000u);check(task_completed_count==1u && task_reference_cps==0.0f);
    begin(0u);request(2u);for(i=0u;i<50u;i++)poll(uptime_ms+20u);
    v=task_reference_cps;request(9u);
    check(task_state==TASK_BRAKING && task_reference_cps==v && task_finish_reason==TASK_FINISH_EARLY_STOP);
    poll(1150u);check(absolute(task_reference_cps-v*0.5f)<0.001f);
    poll(2000u);poll(2500u);
    check(task_state==TASK_DONE && task_finish_reason==TASK_FINISH_EARLY_STOP && task_completed_count==0u && pd_window_active==1u);
    begin(0u);request(2u);limit=task_distance_timeout_ms;
    for(i=0u;i<50u;i++)poll(uptime_ms+20u);
    v=task_reference_cps;poll(limit);
    check(task_state==TASK_BRAKING && task_finish_reason==TASK_FINISH_TIMEOUT && task_reference_cps==v);
    poll(limit+1000u);poll(limit+1500u);
    check(task_state==TASK_DONE && task_finish_reason==TASK_FINISH_TIMEOUT && task_completed_count==0u && pd_window_active==1u);
    runs=task_run_count;poll(limit+10000u);
    check(task_run_count==runs && task_reference_cps==0.0f); /* Timeout never restarts a task. */
    begin(0u);nan.bits=0x7FC00000u;
    check(!task_motion_distance_valid(nan.value) && !task_motion_distance_valid(0.0f) && !task_motion_distance_valid(-20.1f));
    check(task_motion_distance_valid(-20.0f) && task_motion_distance_valid(-5.0f) && task_motion_distance_valid(5.0f) && task_motion_distance_valid(20.0f));
    task_requested_distance_cm=nan.value;request(2u);
    check(task_result==TASK_INVALID && task_state==TASK_BALANCE && task_run_count==0u);
    task_requested_distance_cm=-10.0f;
#ifdef PARALLEL_PID_TRIAL
    speed_parallel_limit_percent=0.0f;
#else
    speed_tilt_limit_deg=0.0f;
#endif
    request(2u);
    check(task_result==TASK_INVALID && task_state==TASK_BALANCE);
    begin(0u);request(2u);task_motion_request(9u);pd_window_cancel();
    check(task_state==TASK_IDLE && task_command==0u && task_reference_cps==0.0f && observed_stby==0u);
    begin(0u);tick(30u);press();
    check(task_state==TASK_FORWARD && task_mode==TASK_MODE_TIMED);
    tick(5600u);press(); /* Keep the fake IMU fresh during the timed task and button handoff. */
    check(task_state==TASK_FORWARD && task_mode==TASK_MODE_TIMED);
    press();
    check(task_state==TASK_BRAKING && task_finish_reason==TASK_FINISH_EARLY_STOP);
    local_sim_pressed=1u;tick(850u);
    check(pd_window_active==0u && task_state==TASK_IDLE && observed_stby==0u);
    local_sim_pressed=0u;tick(100u);
    check(pd_window_active==0u && task_command==0u);
    begin(0u);tick(30u);task_mode=TASK_MODE_DISTANCE;task_state=TASK_DONE;press();
    check(task_state==TASK_FORWARD && task_mode==TASK_MODE_TIMED);
}
#endif

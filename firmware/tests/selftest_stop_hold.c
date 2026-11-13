#include "selftest.h"
#include "speed_loop.h"
#include "task_motion.h"
#include "pd_window.h"
#include "encoder.h"
#include "imu.h"
#include "balance_preview.h"
#include "local_control.h"
static void check_at(uint32_t ok,uint32_t line)
{
    ++selftest_checks;
    if(!ok){if(selftest_failures<32u)selftest_failure_lines[selftest_failures]=120000u+line;++selftest_failures;}
}
#define check(x) check_at((x),__LINE__)
static float magnitude(float v){return v<0.0f?-v:v;}
static void begin(void)
{
    board_motor_off();uptime_ms=0u;encoder_init();imu_init();balance_preview_init();pd_window_init();
    pd_window_active=1u;task_motion_begin_session();task_sim_manual_distance=1u;speed_loop_reset();
    local_sim_pressed=0u;local_control_init();
    balance_kp=5.0f;balance_backward_kp=5.0f;balance_kd=0.13f;
    speed_parallel_kp=25.5f;speed_parallel_ki=0.5f;speed_parallel_limit_percent=100.0f;
    hold_right_boost_counts=0u;hold_common_boost_counts=0u;
    imu_body_pitch_deg=0.0f;imu_pitch_rate_dps=0.0f;
}
static void step(int32_t e1,int32_t e2)
{
    encoder_e1_count=encoder_signed32((uint32_t)encoder_e1_count+(uint32_t)e1);
    encoder_e2_count=encoder_signed32((uint32_t)encoder_e2_count+(uint32_t)e2);
    uptime_ms+=20u;speed_loop_poll();
}
static void request(uint32_t command){task_motion_request(command);task_motion_poll(uptime_ms);}
void stop_hold_selftest_run(void)
{
    uint32_t i;int32_t sign;float offset,reference;
    union{uint32_t u;float f;} nan;
    begin();
    check(position_hold_api==2u && task_stop_ramp_ms==300u);
    check(POSITION_HOLD_EXIT_DEG==5.0f && POSITION_HOLD_EXIT_DPS==35.0f && POSITION_HOLD_EXIT_CMS==8.0f);
    imu_body_pitch_deg=3.5f;imu_pitch_rate_dps=20.0f;
    for(i=0u;i<8u;++i)step(-4,4);
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm<0.5f);
    for(i=0u;i<8u;++i)step(-4,4);
    check(position_hold_offset_cm>0.8f && position_hold_target_cms<0.0f && speed_effective_reference_cps<0.0f);
    check(balance_kp==5.0f && balance_backward_kp==5.0f && speed_parallel_kp==25.5f && speed_parallel_ki==0.5f);
    begin();imu_body_pitch_deg=4.1f;for(i=0u;i<10u;++i)step(-4,4);
    check(position_hold_state==POSITION_HOLD_WAITING);
    imu_body_pitch_deg=4.0f;for(i=0u;i<6u;++i)step(-4,4);
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm>0.8f);
    /* Capture on a user stop while angle and wheel speed are above old entry limits. */
    begin();request(1u);for(i=0u;i<50u;++i)step(0,0);
    check(task_state==TASK_FORWARD && task_reference_cps>0.0f);
    imu_body_pitch_deg=3.5f;imu_pitch_rate_dps=20.0f;speed_filtered_cps=-3.0f*TASK_COUNTS_PER_CM;
    reference=task_reference_cps;request(9u);step(5,-5);
    check(task_state==TASK_BRAKING && position_hold_state==POSITION_HOLD_ACTIVE && task_remaining_ms==280u);
    check(magnitude(position_hold_offset_cm+5.0f/TASK_COUNTS_PER_CM)<0.001f);
    for(i=0u;i<7u;++i)step(0,0);
    check(task_reference_cps>reference*0.44f && task_reference_cps<reference*0.46f);
    for(i=0u;i<7u;++i)step(0,0);
    check(task_state==TASK_SETTLING && task_reference_cps==0.0f && position_hold_state==POSITION_HOLD_ACTIVE);
    offset=position_hold_offset_cm;
    for(i=0u;i<30u;++i){speed_filtered_cps=3.0f*TASK_COUNTS_PER_CM;step(0,0);}
    check(task_state==TASK_SETTLING && position_hold_state==POSITION_HOLD_ACTIVE);
    imu_body_pitch_deg=0.0f;imu_pitch_rate_dps=0.0f;for(i=0u;i<40u;++i)step(0,0);
    check(task_state==TASK_DONE && position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm==offset);
    task_state=TASK_FAILED;step(0,0);
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm==offset && task_reference_cps==0.0f);
    /* Reverse cancellation uses the same early stop handoff. */
    begin();request(2u);for(i=0u;i<50u;++i)step(-4,4);request(9u);step(-4,4);
    check(task_state==TASK_BRAKING && task_reference_cps<0.0f && position_hold_state==POSITION_HOLD_ACTIVE);
    for(i=0u;i<14u;++i)step(0,0);
    check(task_state==TASK_SETTLING && task_reference_cps==0.0f && position_hold_state==POSITION_HOLD_ACTIVE);
    /* Arrive at each signed goal while still rolling: immediate target capture,
     * zero travel reference, then correction toward the original goal. */
    for(sign=-1;sign<=1;sign+=2){
        begin();task_requested_distance_cm=10.0f*(float)sign;request(2u);
        for(i=0u;i<168u;++i)step(4*sign,-4*sign);
        check(task_distance_holding && task_state==TASK_DISTANCE && task_reference_cps==0.0f);
        check(position_hold_state==POSITION_HOLD_ACTIVE && magnitude(position_hold_offset_cm-task_distance_error_cm)<0.001f);
        for(i=0u;i<40u;++i)step(2*sign,-2*sign);
        check(task_distance_holding && task_reference_cps==0.0f && position_hold_state==POSITION_HOLD_ACTIVE);
        check(position_hold_target_cms*(float)sign>0.0f && magnitude(position_hold_offset_cm)>0.6f);
        check(magnitude(speed_effective_reference_cps-position_hold_target_cms*TASK_COUNTS_PER_CM)<0.001f);
    }
    /* Five-second completion captures where the profile ends, before low-speed confirmation. */
    begin();request(1u);for(i=0u;i<249u;++i)step(0,0);step(4,-4);
    check(task_state==TASK_SETTLING && task_finish_reason==TASK_FINISH_TIME && position_hold_state==POSITION_HOLD_ACTIVE);
    step(4,-4);offset=position_hold_offset_cm;
    check(offset<0.0f);
    for(i=0u;i<40u;++i)step(0,0);
    check(task_state==TASK_DONE && position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm==offset);
    /* Disturbance release is retained; only normal safe reacquisition is relaxed. */
    begin();speed_loop_position_hold_at(0.75f);imu_body_pitch_deg=5.1f;step(0,0);
    check(position_hold_state==POSITION_HOLD_DISTURBED && position_hold_target_cms==0.0f);
    imu_body_pitch_deg=0.0f;imu_pitch_rate_dps=36.0f;step(0,0);
    check(position_hold_state==POSITION_HOLD_DISTURBED && position_hold_target_cms==0.0f);
    imu_pitch_rate_dps=0.0f;speed_filtered_cps=12.0f*TASK_COUNTS_PER_CM;step(0,0);
    check(position_hold_state==POSITION_HOLD_DISTURBED && position_hold_target_cms==0.0f);
    begin();uptime_ms=0xfffffff0u;encoder_e1_count=encoder_signed32(0x80000010u);
    encoder_e2_count=encoder_signed32(0x7ffffff0u);speed_loop_reset();speed_loop_position_hold_at(0.75f);step(0,0);
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm==0.75f && position_hold_target_cms<0.0f);
    nan.u=0x7fc00000u;offset=position_hold_offset_cm;speed_loop_position_hold_at(nan.f);
    check(position_hold_state==POSITION_HOLD_ACTIVE && position_hold_offset_cm==offset);
    speed_parallel_limit_percent=0.0f;step(0,0);speed_loop_position_hold_at(1.0f);
    check(position_hold_state==POSITION_HOLD_OFF && position_hold_target_cms==0.0f);
    pd_window_cancel();speed_loop_position_hold_at(1.0f);
    check(!observed_stby && !observed_pwm_a && !observed_pwm_b && position_hold_state==POSITION_HOLD_OFF);
}

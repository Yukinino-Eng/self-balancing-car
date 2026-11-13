#include "selftest.h"
#include "task_motion.h"
#include "pd_window.h"
#include "speed_loop.h"
#include "local_control.h"
#include "param_store.h"
#include "hold_tuning.h"
#include "balance_preview.h"
#include "imu.h"
#include "encoder.h"
#include "pitch_uart.h"
#include "parameter_fixture63.h"
static void check_at(uint32_t ok,uint32_t line)
{
    ++selftest_checks;
    if(!ok){if(selftest_failures<32u)selftest_failure_lines[selftest_failures]=110000u+line;++selftest_failures;}
}
#define check(p) check_at((p),__LINE__)
static uint32_t same(const char *a,const char *b)
{
    while(*a && *a==*b){++a;++b;}
    return *a==*b;
}
static uint32_t bits(float f){union{float f;uint32_t u;}v;v.f=f;return v.u;}
static void begin(void)
{
    board_motor_off();uptime_ms=0u;encoder_init();selftest_reset_guard();
    imu_init();balance_preview_init();pd_window_init();task_motion_init();
    local_sim_pressed=0u;local_control_init();
    pd_window_active=1u;task_motion_begin_session();
}
static void command_checks(void)
{
    uint32_t epoch;
    begin();task_motion_request(2u);task_motion_poll(0u);
    check(task_result==TASK_INVALID && task_state==TASK_BALANCE && task_run_count==0u && task_reference_cps==0.0f);
    begin();epoch=task_cancel_epoch;task_panel_request_id=5u;
    task_panel_expected_epoch=epoch;task_panel_command=2u;task_motion_poll(0u);
    check(task_panel_result_id==5u && task_panel_result==TASK_INVALID && task_state==TASK_BALANCE && task_run_count==0u);
    /* A forbidden host command must not displace an accepted physical press. */
    begin();task_motion_request(1u);task_panel_request_id=6u;
    task_panel_expected_epoch=task_cancel_epoch;task_panel_command=2u;task_motion_poll(0u);
    check(task_panel_result_id==6u && task_panel_result==TASK_INVALID && task_state==TASK_FORWARD && task_run_count==1u);
    pd_window_cancel();
}
static void parameter_checks(void)
{
    uint32_t i,unchanged=1u,data[16];
    pd_window_cancel();local_control_init();param_store_sim_clear();
    for(i=0u;i<512u;i++)param_store_sim_pages[i/256u][i%256u]=parameter_fixture63[i];
    imu_init();balance_preview_init();pd_window_init();param_store_init();
    check(param_store_valid && param_store_sequence==63u && !param_store_dirty);
    hold_tuning_export_record(data);
    for(i=0u;i<16u;i++)check(data[i]==parameter_fixture63[3u+i]);
    check(bits(speed_parallel_kp)==parameter_fixture63[22]);
    check(bits(speed_parallel_ki)==parameter_fixture63[23]);
    check(bits(speed_parallel_limit_percent)==parameter_fixture63[24]);
    check(imu_upright_reference_deg>0.4899f && imu_upright_reference_deg<0.4901f);
    for(i=0u;i<512u;i++)if(param_store_sim_pages[i/256u][i%256u]!=parameter_fixture63[i])unchanged=0u;
    check(unchanged && !observed_stby && !observed_pwm_a && !observed_pwm_b && !pd_window_active);
    balance_kp=6.0f;param_store_mark_dirty();
    pd_window_active=1u;++param_store_request_id;param_store_command=1u;param_store_poll();
    check(param_store_status==STORE_BUSY && param_store_sequence==63u && param_store_dirty);
    pd_window_cancel();++param_store_request_id;param_store_command=1u;param_store_poll();
    check(param_store_status==STORE_SAVED && param_store_sequence==64u && !param_store_dirty);
    balance_preview_init();pd_window_init();param_store_init();
    check(param_store_valid && param_store_sequence==64u && balance_kp==6.0f && !param_store_dirty && !observed_stby);
    unchanged=1u;
    for(i=0u;i<256u;i++)if(param_store_sim_pages[0][i]!=parameter_fixture63[i])unchanged=0u;
    check(unchanged); /* The previous committed page survives the next save. */
    param_store_sim_clear();
}
static void fresh(uint32_t now)
{
    uptime_ms=now;imu_last_sample_ms=now;imu_sample_count=1u;
    imu_seq=2u;imu_valid=1u;imu_fusion_valid=1u;imu_frozen=0u;imu_body_pitch_deg=-1.2346f;
}
static void uart_checks(void)
{
    char text[17];
    union{uint32_t u;float f;} value;
    check(pitch_uart_format(text,0.0f,1u)==16u && same(text,"PITCH=+000.000\r\n"));
    check(pitch_uart_format(text,-1.2346f,1u)==16u && same(text,"PITCH=-001.235\r\n"));
    check(pitch_uart_format(text,180.0f,1u)==16u && same(text,"PITCH=+180.000\r\n"));
    check(pitch_uart_format(text,-180.0f,1u)==16u && same(text,"PITCH=-180.000\r\n"));
    check(pitch_uart_format(text,-0.0001f,1u)==16u && same(text,"PITCH=+000.000\r\n"));
    check(pitch_uart_format(text,180.1f,1u)==15u && same(text,"PITCH=INVALID\r\n"));
    value.u=0x7fc00000u;
    check(pitch_uart_format(text,value.f,1u)==15u && same(text,"PITCH=INVALID\r\n"));
    value.u=0x7f800000u;
    check(pitch_uart_format(text,value.f,1u)==15u && same(text,"PITCH=INVALID\r\n"));
    check(pitch_uart_format(text,1.0f,0u)==15u && same(text,"PITCH=INVALID\r\n"));
    fresh(100u);pitch_uart_init();pitch_uart_poll();
    check(pitch_uart_state==PITCH_UART_READY && pitch_uart_busy && pitch_uart_started==1u && same(pitch_uart_buffer,"PITCH=-001.235\r\n"));
    fresh(101u);imu_body_pitch_deg=80.0f;pitch_uart_poll();
    check(pitch_uart_started==1u && same(pitch_uart_buffer,"PITCH=-001.235\r\n"));
    uptime_ms=102u;pitch_uart_sim_result=1u;pitch_uart_poll();
    check(!pitch_uart_busy && pitch_uart_completed==1u && pitch_uart_started==1u);
    uptime_ms=149u;pitch_uart_poll();check(pitch_uart_started==1u);
    uptime_ms=150u;pitch_uart_poll();
    check(pitch_uart_started==2u && same(pitch_uart_buffer,"PITCH=INVALID\r\n"));
    uptime_ms=151u;pitch_uart_sim_result=2u;pitch_uart_poll();
    check(pitch_uart_state==PITCH_UART_FAILED && pitch_uart_errors==1u && !pitch_uart_busy);
    uptime_ms=1000u;pitch_uart_poll();check(pitch_uart_started==2u && pitch_uart_errors==1u && !observed_stby);
    fresh(200u);pitch_uart_init();pitch_uart_poll();uptime_ms=210u;pitch_uart_poll();
    check(pitch_uart_state==PITCH_UART_FAILED && !pitch_uart_busy && pitch_uart_errors==1u);
    fresh(0xfffffff0u);pitch_uart_init();pitch_uart_poll();
    uptime_ms=0xfffffff2u;pitch_uart_sim_result=1u;pitch_uart_poll();
    uptime_ms=0x21u;pitch_uart_poll();check(pitch_uart_started==1u);
    fresh(0x22u);pitch_uart_poll();check(pitch_uart_started==2u && pitch_uart_busy && !pitch_uart_errors);
    fresh(3000u);pitch_uart_init();pitch_uart_poll();
    fresh(4000u);pitch_uart_sim_result=1u;pitch_uart_poll();pitch_uart_poll();
    check(pitch_uart_started==2u && pitch_uart_completed==1u && pitch_uart_state==PITCH_UART_READY);
    fresh(5000u);imu_seq=3u;pitch_uart_init();pitch_uart_poll();check(same(pitch_uart_buffer,"PITCH=INVALID\r\n"));
    fresh(5100u);imu_frozen=1u;pitch_uart_init();pitch_uart_poll();check(same(pitch_uart_buffer,"PITCH=INVALID\r\n"));
    fresh(5200u);imu_fusion_valid=0u;pitch_uart_init();pitch_uart_poll();check(same(pitch_uart_buffer,"PITCH=INVALID\r\n"));
}
void competition_selftest_run(void)
{
    command_checks();parameter_checks();uart_checks();
}

#include "peripherals.h"
#include "motor_guard.h"
static void direction_frame(uint32_t now)
{
    uint32_t sample=imu_sample_count+1u;
    fresh(now);imu_body_pitch_deg=0.0f;imu_pitch_rate_dps=0.0f;
    imu_sample_count=sample;imu_seq=sample*2u;
    imu_fusion_mode=IMU_FUSION_FUSED;
    if(tof_valid)tof_last_ms=now;
    motor_guard_poll();balance_preview_poll();pd_window_poll();
}
static void direction_step(uint32_t now)
{
    while((uint32_t)(now-uptime_ms)>20u)direction_frame(uptime_ms+20u);
    direction_frame(now);
}
static void direction_begin(void)
{
    board_motor_off();uptime_ms=0u;encoder_init();selftest_reset_guard();
    imu_init();balance_preview_init();pd_window_init();
    local_sim_pressed=0u;local_control_init();tof_valid=0u;
    direction_step(0u);pd_window_command=1u;pd_window_poll();
}
void direction_selftest_run(void)
{
    uint32_t now;
    check(task_direction_reversed==1u);
    direction_begin();task_motion_request(1u);task_motion_poll(0u);direction_step(1000u);
    check(pd_window_active && task_state==TASK_FORWARD && task_reference_cps>0.0f && task_motion_control_reference()<0.0f);
    check(speed_error_cps>0.0f && balance_request_percent>0.0f);
    check(observed_stby && observed_direction_pins==(MOTOR_A1_PIN|MOTOR_B1_PIN));
    /* New forward is old-frame negative travel, not an encoder sign change. */
    encoder_e1_count=704;encoder_e2_count=-704;uptime_ms=1020u;task_motion_poll(1020u);
    check(task_distance_cm>9.9f && task_distance_cm<10.1f);
    check(task_left_distance_cm>9.9f && task_right_distance_cm>9.9f && task_wheel_error_cm==0.0f);
    pd_window_cancel();check(!observed_stby && !observed_pwm_a && !observed_pwm_b);
    /* Smooth cancellation retains the reversed direction and reaches zero. */
    direction_begin();task_motion_request(1u);task_motion_poll(0u);direction_step(1000u);
    task_motion_request(9u);direction_step(1000u);
    check(task_state==TASK_BRAKING && task_motion_control_reference()<0.0f);
    direction_step(2000u);direction_step(2500u);
    check(task_state==TASK_DONE && pd_window_active && task_reference_cps==0.0f);
    /* Real task->PI->PD->actuator path for obstacle automatic reverse. */
    direction_begin();tof_valid=1u;tof_range_mm=150u;tof_last_ms=0u;
    task_motion_request(3u);task_motion_poll(0u);
    check(avoid_state==AVOID_BRAKING && task_reference_cps==0.0f);
    direction_step(300u);direction_step(800u);
    check(avoid_state==AVOID_HOLD && avoid_hold_remaining_ms==5000u);
    direction_step(5799u);check(avoid_state==AVOID_HOLD && avoid_hold_remaining_ms==1u);
    direction_step(5800u);check(avoid_state==AVOID_REVERSE && task_target_distance_cm== -10.0f);
    direction_step(6000u);
    check(task_reference_cps<0.0f && task_motion_control_reference()>0.0f);
    check(observed_stby && balance_request_percent<0.0f && observed_direction_pins==(MOTOR_A2_PIN|MOTOR_B2_PIN));
    encoder_e1_count=-704;encoder_e2_count=704;speed_filtered_cps=0.0f;uptime_ms=6020u;tof_last_ms=6020u;task_motion_poll(6020u);
    check(task_distance_cm< -9.9f && task_distance_cm> -10.1f);
    /* Simulated rest at the reached goal: finish deceleration and dwell. */
    for(now=6040u;now<=6820u;now+=20u){uptime_ms=now;tof_last_ms=now;task_motion_poll(now);}
    check(avoid_state==AVOID_COMPLETE && task_state==TASK_DONE && pd_window_active && avoid_completed_count==1u);
    pd_window_cancel();check(!pd_window_active && !observed_stby && !observed_pwm_a && !observed_pwm_b);
}

static void button_fresh(void)
{
    imu_valid=1u;imu_fusion_valid=1u;imu_fusion_mode=IMU_FUSION_FUSED;
    imu_freeze=0u;imu_frozen=0u;imu_seq+=2u;++imu_sample_count;
    imu_last_sample_ms=uptime_ms;imu_body_pitch_deg=0.0f;imu_pitch_rate_dps=0.0f;
}
static void button_tick(uint32_t count)
{
    while(count--){
        ++uptime_ms;
        if(uptime_ms%10u==0u)button_fresh();
        if(tof_valid)tof_last_ms=uptime_ms;
        local_control_tick();pd_window_tick();
        balance_preview_poll();motor_guard_poll();pd_window_poll();local_control_poll();
    }
}
static void button_reset(uint32_t now,uint32_t held)
{
    board_motor_off();uptime_ms=now;encoder_init();selftest_reset_guard();
    imu_init();balance_preview_init();pd_window_init();param_store_busy=0u;
    local_sim_pressed=held;local_control_init();
    tof_valid=1u;tof_range_mm=500u;tof_last_ms=now;button_fresh();motor_guard_poll();
}
static void button_click(void)
{local_sim_pressed=1u;button_tick(30u);local_sim_pressed=0u;button_tick(30u);}
static void button_hold(uint32_t count)
{local_sim_pressed=1u;button_tick(count);}
static void button_release(void)
{local_sim_pressed=0u;button_tick(30u);}
static void button_wave(uint32_t count,uint32_t *rises,uint32_t *on_ms)
{
    uint32_t previous=observed_led_on;
    *rises=0u;*on_ms=0u;
    while(count--){
        button_tick(1u);
        if(observed_led_on && !previous)++*rises;
        *on_ms+=observed_led_on;previous=observed_led_on;
    }
}
void button_selftest_run(void)
{
    uint32_t rises,on_ms,i,epoch;
    check(local_button_api==2u);
    /* A held power-on key has no action until released and pressed again. */
    button_reset(0u,1u);button_tick(2000u);
    check(local_task_mode==0u && local_press_count==0u && !pd_window_active && !local_command);
    button_release();check(local_state==LOCAL_IDLE && !pd_window_command && !pd_window_active);
    button_wave(2000u,&rises,&on_ms);
    check(local_led_kind==LOCAL_LED_IDLE_BASIC && rises==1u && on_ms==100u);
    /* Exactly one switch after 800ms of debounced press, never a start. */
    button_hold(820u);check(local_task_mode==0u && local_press_count==0u);
    button_tick(1u);
    check(local_task_mode==1u && local_press_count==1u && !pd_window_active && !observed_stby && !pd_window_command);
    button_tick(2000u);check(local_task_mode==1u && local_press_count==1u && !pd_window_active);
    button_release();check(local_state==LOCAL_IDLE && local_press_count==1u && !local_command);
    button_wave(2000u,&rises,&on_ms);
    check(local_led_kind==LOCAL_LED_IDLE_AVOID && rises==2u && on_ms==200u);
    button_hold(850u);button_release();
    check(local_task_mode==0u && local_press_count==2u && !pd_window_active && !observed_stby);
    button_wave(2000u,&rises,&on_ms);check(rises==1u && on_ms==100u);
    /* A short press retains the three-second preparation. */
    button_click();check(local_state==LOCAL_PREPARING && !pd_window_active);
    button_wave(200u,&rises,&on_ms);
    check(local_led_kind==LOCAL_LED_PREPARING && rises==1u && on_ms==100u);
    button_hold(850u);check(local_state==LOCAL_IDLE && local_task_mode==0u && !pd_window_active);
    button_tick(2000u);button_release();
    check(local_task_mode==0u && !pd_window_active && !pd_window_command && !local_command);
    /* A new hold after stop can change mode without power cycling. */
    button_hold(850u);button_release();check(local_task_mode==1u && !pd_window_active);
    button_click();button_tick(3000u);
    check(pd_window_active && task_state==TASK_BALANCE && local_task_mode==1u);
    button_wave(1000u,&rises,&on_ms);
    check(local_led_kind==LOCAL_LED_BALANCE && rises==0u && on_ms==1000u);
    button_click();check(task_state==TASK_APPROACH && avoid_state==AVOID_APPROACH && local_task_mode==1u);
    button_wave(1000u,&rises,&on_ms);
    check(local_led_kind==LOCAL_LED_TASK && rises==1u && on_ms==500u);
    button_hold(850u);
    check(!pd_window_active && !observed_stby && !observed_pwm_a && !observed_pwm_b && local_task_mode==1u);
    button_tick(2000u);button_release();check(local_task_mode==1u && !pd_window_active);
    button_hold(850u);button_release();check(local_task_mode==0u && !pd_window_active);
    /* A running press cannot become a mode switch if another path stops it. */
    button_click();button_tick(3000u);button_hold(400u);pd_window_cancel();button_tick(450u);
    check(local_task_mode==0u && !pd_window_active && !observed_stby);
    button_release();check(!pd_window_active && !pd_window_command && !local_command);
    /* Reject panel arming while the physical key is held for selection. */
    button_reset(0u,0u);button_tick(30u);button_hold(400u);
    local_command=1u;local_control_poll();
    check(local_state==LOCAL_IDLE && local_last_result==1u && !pd_window_command && !pd_window_active);
    button_tick(450u);button_release();check(local_task_mode==1u && !pd_window_active);
    /* Mode changes revoke pending web travel and never consume a short start. */
    button_reset(0u,0u);button_tick(30u);epoch=task_cancel_epoch;
    task_panel_request_id=123u;task_panel_expected_epoch=epoch;task_panel_command=3u;
    button_hold(850u);
    check(local_task_mode==1u && task_cancel_epoch!=epoch && !task_panel_command && !task_command && !pd_window_active);
    button_release();button_tick(3000u);check(!pd_window_active && task_run_count==0u);
    /* Contact bounce shorter than debounce cannot select or start. */
    button_reset(0u,0u);button_tick(30u);
    for(i=0u;i<10u;i++){local_sim_pressed=1u;button_tick(10u);local_sim_pressed=0u;button_tick(10u);}
    check(local_press_count==0u && local_task_mode==0u && !pd_window_active && local_state==LOCAL_IDLE);
    param_store_busy=1u;button_hold(850u);button_release();
    check(local_task_mode==0u && !pd_window_active && !pd_window_command);
    param_store_busy=0u;button_hold(850u);button_release();check(local_task_mode==1u && !pd_window_active);
    /* Unsigned timestamps preserve hold and LED cadence over the wrap. */
    button_reset(0xffffff00u,0u);button_tick(30u);button_hold(850u);button_release();
    check(local_task_mode==1u && local_press_count==1u && !pd_window_active);
    button_wave(2000u,&rises,&on_ms);check(rises==2u && on_ms==200u && local_led_kind==LOCAL_LED_IDLE_AVOID);
    /* Fault and preparation rejection have a distinct four-pulse pattern. */
    button_reset(0u,0u);button_tick(30u);pd_window_status=PD_FAULT;button_tick(1u);
    button_wave(1600u,&rises,&on_ms);
    check(local_led_kind==LOCAL_LED_FAULT && rises==4u && on_ms==400u && !pd_window_active);
    pd_window_status=PD_IDLE;local_state=LOCAL_REJECTED;button_tick(1u);
    button_wave(1600u,&rises,&on_ms);check(local_led_kind==LOCAL_LED_FAULT && rises==4u && on_ms==400u);
    button_click();check(local_state==LOCAL_PREPARING && local_led_kind==LOCAL_LED_PREPARING && !pd_window_active);
    pd_window_cancel();button_tick(1u);check(local_led_kind==LOCAL_LED_IDLE_BASIC && !observed_stby);
}

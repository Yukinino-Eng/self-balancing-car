#include "local_control.h"
#ifdef PD_HOLD_TEST
#include "board.h"
#include "runtime.h"
#include "motor_guard.h"
#include "pd_window.h"
#include "param_store.h"
#include "task_motion.h"
#define BUTTON_PIN (1u<<10)
#define DEBOUNCE_MS 20u
#define PREPARE_MS 3000u
const volatile uint32_t local_button_api=2u;
volatile uint32_t local_led_kind;
volatile uint32_t local_command,local_state,local_remaining_ms;
volatile uint32_t local_run_source,local_press_count,local_last_result;
volatile uint32_t local_alive_counter;
#ifdef TASK_HAS_OBSTACLE
volatile uint32_t local_task_mode;
#endif
#ifdef LEARN_SIMULATOR
volatile uint32_t local_sim_pressed;
#endif
static uint32_t stable_pressed,candidate_pressed,candidate_ms,released;
static uint32_t prepare_ms,initialized;
static uint32_t press_ms,press_armed,long_handled,press_can_switch;
static uint32_t alive_seen,alive_ms,led_epoch;
void local_control_cancel(void)
{
    local_state=LOCAL_IDLE;local_remaining_ms=0u;local_run_source=0u;
    local_command=0u;local_led_kind=0u;
}
void local_control_init(void)
{
#ifndef LEARN_SIMULATOR
    RCC->APB2ENR|=RCC_APB2ENR_IOPBEN;
    GPIOB->BSRR=BUTTON_PIN;
    GPIOB->CRH=(GPIOB->CRH & ~(0xFu<<8)) | (0x8u<<8);
#endif
#ifdef TASK_HAS_OBSTACLE
    local_task_mode=0u;
#endif
    local_control_cancel();local_press_count=0u;local_last_result=0u;
    stable_pressed=1u;candidate_pressed=1u;candidate_ms=uptime_ms;released=0u;
    press_armed=0u;long_handled=0u;press_can_switch=0u;
    local_alive_counter=0u;alive_seen=0u;alive_ms=uptime_ms;
    led_epoch=uptime_ms;initialized=1u;
}
static uint32_t stopped_for_mode(void)
{
    return !pd_window_active && local_state!=LOCAL_PREPARING && local_state!=LOCAL_REQUESTED &&
           !observed_stby && !observed_pwm_a && !observed_pwm_b &&
           !param_store_busy && !local_command && !pd_window_command;
}
static void arm(uint32_t source)
{
    if(stable_pressed || pd_window_active || param_store_busy){local_last_result=1u;return;}
    local_state=LOCAL_PREPARING;local_remaining_ms=PREPARE_MS;
    prepare_ms=uptime_ms;local_run_source=source;local_last_result=0u;
    alive_seen=local_alive_counter;alive_ms=uptime_ms;
}
static void stop(void)
{
    pd_window_cancel();
}
static void short_press(void)
{
    if(local_state==LOCAL_PREPARING || local_state==LOCAL_REQUESTED)stop();
    else if(!pd_window_active){if(press_can_switch)arm(2u);else stop();}
#ifdef TASK_HAS_OBSTACLE
    else if(task_state==TASK_FAILED)task_motion_request(9u);
    else if(task_motion_avoid_active())task_motion_request(9u);
    else if(local_task_mode==1u && (task_state==TASK_BALANCE || task_state==TASK_DONE))task_motion_request(3u);
#endif
    else if(task_state==TASK_BALANCE || task_state==TASK_DONE)task_motion_request(1u);
    else if(task_state==TASK_FORWARD || task_state==TASK_DISTANCE)task_motion_request(9u);
}
static void long_press(void)
{
#ifdef TASK_HAS_OBSTACLE
    if(press_can_switch && stopped_for_mode()){
        /* Revoke staged travel, keep outputs off, then change only the mode.
         * A press that began in a running/preparing state can only stop. */
        stop();local_task_mode=local_task_mode==1u?0u:1u;local_last_result=0u;
        return;
    }
#endif
    stop();
}
static uint32_t led_kind(void)
{
    if(local_state==LOCAL_PREPARING || local_state==LOCAL_REQUESTED)return LOCAL_LED_PREPARING;
    if(pd_window_active){
#ifdef TASK_HAS_OBSTACLE
        if(task_state==TASK_FAILED || avoid_state==AVOID_FAILED || avoid_state==AVOID_ABORTING)return LOCAL_LED_FAULT;
        if(task_motion_avoid_active())return LOCAL_LED_TASK;
#endif
        if(task_state!=TASK_BALANCE && task_state!=TASK_DONE)return LOCAL_LED_TASK;
        return LOCAL_LED_BALANCE;
    }
    if(local_state==LOCAL_REJECTED || pd_window_status==PD_FAULT)return LOCAL_LED_FAULT;
#ifdef TASK_HAS_OBSTACLE
    if(local_task_mode==1u)return LOCAL_LED_IDLE_AVOID;
#endif
    return LOCAL_LED_IDLE_BASIC;
}
static uint32_t flashes(uint32_t phase,uint32_t count)
{
    return phase/200u<count && phase%200u<100u;
}
static void update_led(void)
{
    uint32_t kind=led_kind(),phase;
    if(kind!=local_led_kind){local_led_kind=kind;led_epoch=uptime_ms;}
    phase=(uint32_t)(uptime_ms-led_epoch);
    if(kind==LOCAL_LED_PREPARING)board_led_set(phase%200u<100u);
    else if(kind==LOCAL_LED_BALANCE)board_led_set(1u);
    else if(kind==LOCAL_LED_TASK)board_led_set(phase%1000u<500u);
    else if(kind==LOCAL_LED_FAULT)board_led_set(flashes(phase%1600u,4u));
    else board_led_set(flashes(phase%2000u,kind==LOCAL_LED_IDLE_AVOID?2u:1u));
}
void local_control_tick(void)
{
    uint32_t pressed,elapsed;
    if(!initialized)return;
    if(local_alive_counter!=alive_seen){alive_seen=local_alive_counter;alive_ms=uptime_ms;}
    if(local_run_source==1u && (pd_window_active || local_state==LOCAL_PREPARING || local_state==LOCAL_REQUESTED) &&
       (uint32_t)(uptime_ms-alive_ms)>=2500u){stop();local_last_result=5u;}
#ifdef LEARN_SIMULATOR
    pressed=local_sim_pressed!=0u;
#else
    pressed=(GPIOB->IDR & BUTTON_PIN)==0u;
#endif
    if(pressed!=candidate_pressed){candidate_pressed=pressed;candidate_ms=uptime_ms;}
    if((uint32_t)(uptime_ms-candidate_ms)>=DEBOUNCE_MS && stable_pressed!=candidate_pressed){
        stable_pressed=candidate_pressed;
        if(!stable_pressed){
            released=1u;
            if(press_armed && !long_handled){++local_press_count;short_press();}
            press_armed=0u;
        }else if(released){
            press_ms=uptime_ms;press_armed=1u;long_handled=0u;press_can_switch=stopped_for_mode();
        }
    }
    if(stable_pressed && press_armed && !long_handled && (uint32_t)(uptime_ms-press_ms)>=LOCAL_LONG_PRESS_MS){
        long_handled=1u;++local_press_count;long_press();
    }
    if(local_state==LOCAL_PREPARING){
        elapsed=(uint32_t)(uptime_ms-prepare_ms);
        if(elapsed<PREPARE_MS)local_remaining_ms=PREPARE_MS-elapsed;
        else{
            local_remaining_ms=0u;
            if(stable_pressed || param_store_busy || motor_guard_check(uptime_ms)!=GUARD_READY ||
               !(motor_guard_body_deg>=-2.0f && motor_guard_body_deg<=2.0f)){
                local_state=LOCAL_REJECTED;local_run_source=0u;local_last_result=2u;
            }else{pd_window_command=1u;local_state=LOCAL_REQUESTED;}
        }
    }
    if(pd_window_active)local_state=LOCAL_RUNNING;
    update_led();
}
void local_control_poll(void)
{
    uint32_t command,saved=__get_PRIMASK();
    __disable_irq();command=local_command;local_command=0u;
    if(command==9u)stop();
    else if(command==1u){if(local_state==LOCAL_PREPARING || local_state==LOCAL_REQUESTED)local_last_result=1u;else arm(1u);}
    else if(command!=0u)local_last_result=3u;
    if(local_state==LOCAL_REQUESTED && pd_window_command==0u){
        if(pd_window_active)local_state=LOCAL_RUNNING;
        else{local_state=LOCAL_REJECTED;local_run_source=0u;local_last_result=4u;}
    }
    __set_PRIMASK(saved);
}
#endif

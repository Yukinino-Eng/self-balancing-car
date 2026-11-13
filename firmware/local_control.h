#ifndef LOCAL_CONTROL_H
#define LOCAL_CONTROL_H
#include <stdint.h>
#include "task_motion.h"
#ifdef TASK_HAS_OBSTACLE
extern volatile uint32_t local_task_mode;
#endif
#ifdef PD_HOLD_TEST
enum { LOCAL_IDLE=0, LOCAL_PREPARING=1, LOCAL_RUNNING=2, LOCAL_REJECTED=3, LOCAL_REQUESTED=4 };
enum { LOCAL_LED_IDLE_BASIC=1,LOCAL_LED_IDLE_AVOID=2,LOCAL_LED_PREPARING=3,
       LOCAL_LED_BALANCE=4,LOCAL_LED_TASK=5,LOCAL_LED_FAULT=6 };
#define LOCAL_LONG_PRESS_MS 800u
extern const volatile uint32_t local_button_api;
extern volatile uint32_t local_led_kind;
extern volatile uint32_t local_command, local_state, local_remaining_ms;
extern volatile uint32_t local_run_source, local_press_count, local_last_result;
extern volatile uint32_t local_alive_counter;
#ifdef LEARN_SIMULATOR
extern volatile uint32_t local_sim_pressed;
#endif
void local_control_init(void);
void local_control_tick(void);
void local_control_poll(void);
void local_control_cancel(void);
#endif
#endif

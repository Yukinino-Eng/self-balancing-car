#ifndef BOARD_H
#define BOARD_H

#include "stm32f10x.h"

#define PWM_PERIOD_COUNTS 400u
/* 双轮首次联动只允许已约定的小占空比和短时间。 */
#define MOTOR_A1_PIN (1u << 14)
#define MOTOR_A2_PIN (1u << 15)
#define MOTOR_B1_PIN (1u << 13)
#define MOTOR_B2_PIN (1u << 12)
#define MOTOR_DIRECTION_PINS 0xF000u
#define MOTOR_STBY_PIN (1u << 8)

/* These mirrors can be viewed in Watch, including in the simulator. */
extern const uint32_t build_is_simulator;
extern volatile uint32_t observed_stby;
extern volatile uint32_t observed_direction_pins;
extern volatile uint32_t observed_pwm_a;
extern volatile uint32_t observed_pwm_b;
extern volatile uint32_t watchdog_started;

void board_init(void);
void board_tick_init(void);
void board_motor_off(void);
#ifdef PD_HOLD_TEST
#include "hold_config.h"
#define HOLD_RIGHT_BOOST_PERMILLE 20u
#define HOLD_RIGHT_BOOST_COUNTS ((PWM_PERIOD_COUNTS * HOLD_RIGHT_BOOST_PERMILLE + 500u) / 1000u)
#define HOLD_RIGHT_BOOST_MAX_BASE_PERCENT 5u
extern volatile float hold_right_boost_percent;
extern volatile uint32_t hold_right_boost_counts;
extern volatile uint32_t hold_right_boost_max_base_percent;
void board_hold_start(uint32_t command, uint32_t duty_percent);
void board_hold_start_counts(uint32_t command, uint32_t base_counts, int32_t sync_counts);
extern volatile uint32_t hold_common_boost_counts, hold_actuator_limited;
extern volatile int32_t hold_sync_applied_counts,hold_sync_saturation_error_counts;
extern volatile float hold_common_boost_percent;
#endif

void board_led_toggle(void);
void board_led_set(uint32_t on);
extern volatile uint32_t observed_led_on;
void board_watchdog_start(void);
void board_watchdog_feed(void);

#endif

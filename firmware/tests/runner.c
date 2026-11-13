#include "selftest.h"
#include "motor_guard.h"
#include "pd_window.h"
#include "imu.h"
#include "balance_preview.h"
volatile uint32_t selftest_checks,selftest_failures,selftest_finished;
volatile uint32_t selftest_failure_lines[32];
void park_selftest_run(void);
void stop_hold_selftest_run(void);
#ifdef REVIEW_ONLY_SELFTEST
void distance_selftest_run(void);
void competition_selftest_run(void);
void direction_selftest_run(void);
void button_selftest_run(void);
#endif
void selftest_reset_guard(void){motor_guard_init();board_motor_off();}
void motor_selftest_run(void)
{
    selftest_checks=0u;selftest_failures=0u;selftest_finished=0u;
#ifdef STOP_HOLD_ONLY_SELFTEST
    stop_hold_selftest_run();
#else
    park_selftest_run();
#ifdef REVIEW_ONLY_SELFTEST
    distance_selftest_run();
    competition_selftest_run();
    direction_selftest_run();
    button_selftest_run();
#endif
#endif
    board_motor_off();balance_preview_init();pd_window_init();imu_init();
    selftest_finished=1u;
}

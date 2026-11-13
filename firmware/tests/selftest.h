#ifndef COMPETITION_SELFTEST_H
#define COMPETITION_SELFTEST_H
#include "board.h"
#include "runtime.h"
extern volatile uint32_t selftest_checks,selftest_failures,selftest_finished;
extern volatile uint32_t selftest_failure_lines[32];
void selftest_reset_guard(void);
void motor_selftest_run(void);
#endif

#ifndef BALANCE_PREVIEW_H
#define BALANCE_PREVIEW_H

#include <stdint.h>

#define BALANCE_DEG_TO_RAD 0.01745329252f
#define BALANCE_RAD_TO_DEG 57.29577951f
/* P14: Hold Kp=4; Ground Kp=2; fixed-load and older targets retain 1. */
#if defined(PD_HOLD_TEST)
#define BALANCE_GAIN_TRIAL 1u
#define BALANCE_DEFAULT_KP 4.0f
#elif defined(PD_GROUND_TEST) && !defined(PD_LOAD_TEST)
#define BALANCE_GAIN_TRIAL 1u
#define BALANCE_DEFAULT_KP 2.0f
#else
#define BALANCE_GAIN_TRIAL 0u
#define BALANCE_DEFAULT_KP 1.0f
#endif
#define BALANCE_DEFAULT_KD 0.05f
extern const volatile uint32_t balance_gain_trial;

#ifdef PD_HOLD_TEST
#include "hold_config.h"
#define BALANCE_OUTPUT_LIMIT_PERCENT ((float)HOLD_OUTPUT_LIMIT_PERCENT)
#else
#define BALANCE_OUTPUT_LIMIT_PERCENT 10.0f
#endif
#include "tilt_config.h"
#define BALANCE_TILT_LIMIT_RAD (CAR_BALANCE_MAX_TILT_DEG * BALANCE_DEG_TO_RAD)
#define BALANCE_SAMPLE_MAX_AGE_MS 100u

enum {
    BALANCE_WAITING = 0,
    BALANCE_PREVIEW_READY = 1,
    BALANCE_IMU_INVALID = 2,
    BALANCE_TILT_LIMIT = 3,
    BALANCE_CONFIG_INVALID = 4
};

/* 演示参数：Kp单位%/deg，Kd单位%/(deg/s)，不是实车已调好的增益。 */
extern volatile float balance_kp, balance_kd;
#ifdef PD_HOLD_TEST
extern volatile float balance_backward_kp;
#endif
extern const volatile uint32_t balance_preview_only;
extern volatile float balance_pitch_rad, balance_rate_rad_s;
extern volatile float balance_p_percent, balance_d_percent;
extern volatile float balance_request_percent;
extern volatile int32_t balance_request_direction;
extern volatile uint32_t balance_status, balance_output_limited;
extern volatile uint32_t balance_update_count;

void balance_preview_init(void);
/* 控制内部输入采用前倾为正的rad、rad/s；仅计算，不接硬件输出。 */
void balance_preview_evaluate(float forward_pitch_rad, float forward_rate_rad_s,
                              uint32_t valid);
void balance_preview_poll(void);

#endif

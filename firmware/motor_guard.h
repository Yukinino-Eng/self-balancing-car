#ifndef MOTOR_GUARD_H
#define MOTOR_GUARD_H
#include <stdint.h>
#define MOTOR_GUARD_MAX_AGE_MS 30u
#include "tilt_config.h"
#define MOTOR_GUARD_MAX_TILT_DEG CAR_BALANCE_MAX_TILT_DEG
enum {
    GUARD_WAITING = 0, GUARD_READY = 1, GUARD_IMU_INVALID = 2,
    GUARD_STALE = 3, GUARD_TILT = 4, GUARD_FROZEN = 5
};
/* 观察变量，不是使能开关。保护供Joint短脉冲和PD限时窗口使用。 */
extern volatile uint32_t motor_guard_state, motor_guard_age_ms;
extern volatile float motor_guard_body_deg;
typedef struct {
    uint32_t valid_input, frozen, seq, sample_count, sample_ms;
    uint32_t imu_valid, fusion_valid, error, error_count, reject_flags, input_dt_ms;
    float body_deg, rate_dps, ax_g, ay_g, az_g, norm2_g2;
    uint32_t fusion_mode, gyro_only_age_ms, gyro_only_count;
} motor_guard_snapshot_t;
/* 返回上次完整提交的缓存，不在SysTick中读取发布到一半的IMU。 */
void motor_guard_copy_snapshot(volatile motor_guard_snapshot_t *out);
void motor_guard_init(void);
void motor_guard_poll(void);
uint32_t motor_guard_check(uint32_t now_ms);
/* 主循环提交完整姿态快照；模拟器用相同入口提供测试帧。 */
void motor_guard_update(float body_deg, uint32_t sample_count, uint32_t valid,
                        uint32_t frozen, uint32_t seq, uint32_t timestamp_ms);
#endif

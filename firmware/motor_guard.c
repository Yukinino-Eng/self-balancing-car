#include "motor_guard.h"
#include "board.h"
#include "imu.h"
#include "runtime.h"

volatile uint32_t motor_guard_state = GUARD_WAITING;
volatile uint32_t motor_guard_age_ms = 0u;
volatile float motor_guard_body_deg = 0.0f;
static volatile uint32_t snapshot_reason = GUARD_WAITING;
static volatile uint32_t snapshot_ms;
static volatile motor_guard_snapshot_t snapshot_detail;

void motor_guard_init(void)
{
    const motor_guard_snapshot_t empty = {0};
    snapshot_detail = empty;
    snapshot_reason = GUARD_WAITING;
    snapshot_ms = 0u;
    motor_guard_state = GUARD_WAITING;
    motor_guard_age_ms = 0u;
    motor_guard_body_deg = 0.0f;
}

void motor_guard_update(float body, uint32_t count, uint32_t valid,
                        uint32_t frozen, uint32_t seq, uint32_t stamp)
{
    uint32_t reason, saved_primask;
    motor_guard_snapshot_t detail;
    if (frozen != 0u) { reason = GUARD_FROZEN; }
    else if (count == 0u) { reason = GUARD_WAITING; }
    else if (valid == 0u || (seq & 1u) != 0u ||
             !(body >= -180.0f && body <= 180.0f)) {
        reason = GUARD_IMU_INVALID;   /* 同时拒绝NaN和无穷。 */
    } else if (body < -MOTOR_GUARD_MAX_TILT_DEG ||
               body > MOTOR_GUARD_MAX_TILT_DEG) { reason = GUARD_TILT; }
    else { reason = GUARD_READY; }
    /* 仅主循环调用。短临界区发布缓存，SysTick不会读到半份数据。
       中断读取上一份完整样本，不因IMU正在发布奇数seq而误触发。 */
    detail.valid_input = valid; detail.frozen = frozen; detail.seq = seq;
    detail.sample_count = count; detail.sample_ms = stamp;
    detail.imu_valid = imu_valid; detail.fusion_valid = imu_fusion_valid;
    detail.error = imu_error; detail.error_count = imu_error_count;
    detail.reject_flags = imu_fusion_reject_flags;
    detail.input_dt_ms = imu_fusion_input_dt_ms;
    detail.body_deg = body; detail.rate_dps = imu_pitch_rate_dps;
    detail.ax_g = imu_ax_g; detail.ay_g = imu_ay_g; detail.az_g = imu_az_g;
    detail.norm2_g2 = imu_accel_norm2_g2;
    detail.fusion_mode = imu_fusion_mode;
    detail.gyro_only_age_ms = imu_gyro_only_age_ms;
    detail.gyro_only_count = imu_gyro_only_count;
    saved_primask = __get_PRIMASK();
    __disable_irq();
    snapshot_ms = stamp;
    motor_guard_body_deg = body;
    snapshot_detail = detail;
    snapshot_reason = reason;
    __set_PRIMASK(saved_primask);
}

void motor_guard_copy_snapshot(volatile motor_guard_snapshot_t *out)
{
    uint32_t saved_primask = __get_PRIMASK();
    __disable_irq();
    *out = snapshot_detail;
    __set_PRIMASK(saved_primask);
}

uint32_t motor_guard_check(uint32_t now)
{
    uint32_t reason = snapshot_reason;
    uint32_t age = (uint32_t)(now - snapshot_ms);
    motor_guard_age_ms = age;
    if (reason == GUARD_READY && age > MOTOR_GUARD_MAX_AGE_MS) {
        reason = GUARD_STALE;
    }
    motor_guard_state = reason;
    return reason;
}

void motor_guard_poll(void)
{
    uint32_t seq = imu_seq;
    /* imu_poll与本函数在同一主循环。偶数seq表示完整发布；
       冻结、通信失败和融合无效都不能继续当作正常输入。 */
    motor_guard_update(imu_body_pitch_deg, imu_sample_count,
                        imu_valid && imu_fusion_valid,
                        imu_freeze || imu_frozen, seq, imu_last_sample_ms);
    (void)motor_guard_check(uptime_ms);
}

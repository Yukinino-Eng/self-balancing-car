#include "balance_preview.h"
#include "imu.h"
#include "runtime.h"
#ifdef PD_HOLD_TEST
#include "pd_window.h"
#include "speed_loop.h"
#endif

const volatile uint32_t balance_gain_trial = BALANCE_GAIN_TRIAL;
volatile float balance_kp = BALANCE_DEFAULT_KP;
volatile float balance_kd = BALANCE_DEFAULT_KD;
#ifdef PD_HOLD_TEST
volatile float balance_backward_kp = HOLD_BACKWARD_DEFAULT_KP;
#endif
/* 目标连接方式的观察标记，不是使能开关。 */
#ifdef PD_WINDOW_TEST
const volatile uint32_t balance_preview_only = 0u;
#else
const volatile uint32_t balance_preview_only = 1u;
#endif
volatile float balance_pitch_rad = 0.0f, balance_rate_rad_s = 0.0f;
volatile float balance_p_percent = 0.0f, balance_d_percent = 0.0f;
volatile float balance_request_percent = 0.0f;
volatile int32_t balance_request_direction = 0;
volatile uint32_t balance_status = BALANCE_WAITING;
volatile uint32_t balance_output_limited = 0u;
volatile uint32_t balance_update_count = 0u;
static uint32_t last_sample_count;

static void clear_request(uint32_t status)
{
    balance_p_percent = 0.0f;
    balance_d_percent = 0.0f;
    balance_request_percent = 0.0f;
    balance_request_direction = 0;
    balance_output_limited = 0u;
    balance_status = status;
}

void balance_preview_init(void)
{
    balance_kp = BALANCE_DEFAULT_KP;
    balance_kd = BALANCE_DEFAULT_KD;
#ifdef PD_HOLD_TEST
    balance_backward_kp = HOLD_BACKWARD_DEFAULT_KP;
    speed_loop_init();
#endif
    balance_pitch_rad = 0.0f;
    balance_rate_rad_s = 0.0f;
    balance_update_count = 0u;
    last_sample_count = 0u;
    clear_request(BALANCE_WAITING);
}

void balance_preview_evaluate(float forward_pitch_rad, float forward_rate_rad_s,
                              uint32_t valid)
{
    float kp = balance_kp;
    float kd = balance_kd;
    float request, error_pitch_rad;
#ifdef PD_HOLD_TEST
    float limit = (float)hold_output_limit_percent;
    float backward_limit = (float)hold_backward_limit_percent;
    float backward_kp = balance_backward_kp;
    float max_kp = HOLD_MAX_KP, max_kd = HOLD_MAX_KD;
#else
    float limit = BALANCE_OUTPUT_LIMIT_PERCENT;
    float backward_limit = BALANCE_OUTPUT_LIMIT_PERCENT;
    float max_kp = 5.0f, max_kd = 0.5f;
#endif
    ++balance_update_count;
    clear_request(BALANCE_IMU_INVALID);
    /* 比较同时拒绝NaN/无穷；角速度范围沿用当前IMU量程。 */
    if (valid == 0u ||
        !(forward_pitch_rad >= -3.14159266f && forward_pitch_rad <= 3.14159266f) ||
        !(forward_rate_rad_s >= -IMU_GYRO_MAX_DPS * BALANCE_DEG_TO_RAD &&
          forward_rate_rad_s <= IMU_GYRO_MAX_DPS * BALANCE_DEG_TO_RAD)) {
        return;
    }
    if (!(limit >= 1.0f && limit <= BALANCE_OUTPUT_LIMIT_PERCENT)) {
        balance_status = BALANCE_CONFIG_INVALID; return;
    }
#ifdef PD_HOLD_TEST
    if (!(backward_limit >= 1.0f && backward_limit <= (float)HOLD_BACKWARD_LIMIT_PERCENT &&
          backward_kp >= 0.0f && backward_kp <= HOLD_MAX_KP)) {
        balance_status = BALANCE_CONFIG_INVALID; return;
    }
#endif
    balance_pitch_rad = forward_pitch_rad;
    balance_rate_rad_s = forward_rate_rad_s;
    if (!(kp >= 0.0f && kp <= max_kp && kd >= 0.0f && kd <= max_kd)) {
        balance_status = BALANCE_CONFIG_INVALID;
        return;
    }
    if (forward_pitch_rad < -BALANCE_TILT_LIMIT_RAD ||
        forward_pitch_rad > BALANCE_TILT_LIMIT_RAD) {
        balance_status = BALANCE_TILT_LIMIT;
        return;
    }
    /* 参数从度制换为弧度制：前倾正，建议轮子向前追也为正。 */
#ifdef PD_HOLD_TEST
#ifdef PARALLEL_PID_TRIAL
    error_pitch_rad=forward_pitch_rad; /* Fixed upright reference. */
#else
    error_pitch_rad=forward_pitch_rad+speed_target_body_deg*BALANCE_DEG_TO_RAD;
#endif
    /* Select gains by error around the moving target, keeping P continuous. */
    if(error_pitch_rad<0.0f) kp=backward_kp;
#else
    error_pitch_rad=forward_pitch_rad;
#endif
    balance_p_percent = (kp * BALANCE_RAD_TO_DEG) * error_pitch_rad;
    balance_d_percent = (kd * BALANCE_RAD_TO_DEG) * forward_rate_rad_s;
    request = balance_p_percent + balance_d_percent;
#ifdef PARALLEL_PID_TRIAL
    request += speed_loop_parallel_apply(request,limit,backward_limit);
#endif
    if (request > limit) {
        request = limit;
        balance_output_limited = 1u;
    } else if (request < -backward_limit) {
        request = -backward_limit;
        balance_output_limited = 1u;
    }
    balance_request_percent = request;
    balance_request_direction = request > 0.0f ? 1 : request < 0.0f ? -1 : 0;
    balance_status = BALANCE_PREVIEW_READY;
    /* 到此为止：不写motor_command/STBY/PWM，IMU目标继续锁住电机。 */
}

void balance_preview_poll(void)
{
    uint32_t sample = imu_sample_count;
    uint32_t age = (uint32_t)(uptime_ms - imu_last_sample_ms);
    if (sample == 0u) {
        clear_request(BALANCE_WAITING);
        return;
    }
    if (imu_valid == 0u || imu_fusion_valid == 0u ||
        imu_freeze != 0u || imu_frozen != 0u || age > BALANCE_SAMPLE_MAX_AGE_MS) {
        if (balance_status != BALANCE_IMU_INVALID) {
            balance_preview_evaluate(0.0f, 0.0f, 0u);
        }
        return;
    }
    if (sample == last_sample_count) { return; }
    last_sample_count = sample;
#ifdef PD_HOLD_TEST
    speed_loop_poll();
#endif
    /* 实物已确认body前倾为负；只在进入控制模块时转换坐标和单位。 */
    balance_preview_evaluate(-imu_body_pitch_deg * BALANCE_DEG_TO_RAD,
                             -imu_pitch_rate_dps * BALANCE_DEG_TO_RAD, 1u);
}

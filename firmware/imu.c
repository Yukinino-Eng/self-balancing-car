#include "imu.h"
#include "runtime.h"
#include <math.h>

volatile uint32_t imu_state = IMU_WAITING;
volatile uint32_t imu_who_am_i = 0u;
volatile uint32_t imu_error = IMU_OK;
volatile uint32_t imu_error_count = 0u;
volatile uint32_t imu_i2c_stage = 0u;
volatile uint32_t imu_sample_count = 0u;
volatile uint32_t imu_last_sample_ms = 0u;
volatile uint32_t imu_valid = 0u;
volatile uint32_t imu_seq = 0u;
volatile uint32_t imu_freeze = 0u;
volatile uint32_t imu_frozen = 0u;
volatile int32_t imu_ax_raw = 0, imu_ay_raw = 0, imu_az_raw = 0;
volatile int32_t imu_gx_raw = 0, imu_gy_raw = 0, imu_gz_raw = 0;
volatile int32_t imu_temperature_raw = 0;
volatile float imu_ax_g = 0.0f, imu_ay_g = 0.0f, imu_az_g = 0.0f;
volatile float imu_gx_dps = 0.0f, imu_gy_dps = 0.0f, imu_gz_dps = 0.0f;
volatile float imu_temperature_c = 0.0f;
volatile float imu_roll_deg = 0.0f, imu_pitch_deg = 0.0f;
volatile uint32_t imu_accel_angle_valid = 0u;
/* 本车芯片面朝下；偏差来自2026-10-17、约23.08摄氏度的静止测量。 */
volatile uint32_t imu_gyro_config_readback = 0xFFFFFFFFu;
const float imu_gyro_bias_x_dps = -1.822916f;
const float imu_gyro_bias_y_dps =  0.087446f;
const float imu_gyro_bias_z_dps = -0.003930f;
volatile float imu_gx_corrected_dps = 0.0f;
volatile float imu_gy_corrected_dps = 0.0f;
volatile float imu_gz_corrected_dps = 0.0f;
volatile float imu_pitch_rate_dps = 0.0f;
volatile float imu_pitch_fused_deg = 0.0f;
volatile uint32_t imu_fusion_valid = 0u;
volatile uint32_t imu_fusion_dt_ms = 0u;
volatile uint32_t imu_fusion_seed_count = 0u;
/* 用户2026-10-18手扶直立测得，初始参考，后续平衡时仍需微调。 */
/* User requested zero shifts: +3.18 deg, then another +1.50 deg.
   Absolute fused reference: -3.69 + 3.18 + 1.50 - 0.50 = +0.49 deg. */
const volatile float imu_upright_reference_deg = 0.49f;
volatile float imu_body_pitch_deg = 0.0f;
#ifdef PD_HOLD_TEST
volatile uint32_t imu_gyro_predict_limit_ms = IMU_GYRO_PREDICT_DEFAULT_MS;
#endif
volatile uint32_t imu_fusion_reject_flags = IMU_REJECT_WAITING;
volatile uint32_t imu_fusion_input_dt_ms = 0u;
volatile float imu_accel_norm2_g2 = 0.0f;
volatile uint32_t imu_fusion_mode = IMU_FUSION_INVALID;
volatile uint32_t imu_gyro_only_age_ms = 0u, imu_gyro_only_count = 0u;
volatile uint32_t imu_gyro_only_max_age_ms = 0u;
#ifdef PD_GROUND_TEST
const volatile uint32_t imu_gyro_only_enabled = 1u;
#else
const volatile uint32_t imu_gyro_only_enabled = 0u;
#endif
static uint32_t fusion_previous_ms;
#ifdef PD_GROUND_TEST
static uint32_t fusion_trusted_ms, fusion_has_history;
#endif

void imu_fusion_invalidate(void)
{
    /* 保留上次角度供查看；valid=0时不可作为实时姿态使用。 */
    imu_fusion_valid = 0u;
    imu_fusion_mode = IMU_FUSION_INVALID;
    imu_fusion_dt_ms = 0u;
}

void imu_pitch_filter_update(float accel_pitch_deg, float rate_dps,
                             uint32_t trusted, uint32_t timestamp_ms)
{
    uint32_t dt_ms;
    float dt;
    float alpha;
    float predicted;
    /* 原门槛不变；分项记录当前帧为什么不可信。 */
    dt_ms = (uint32_t)(timestamp_ms - fusion_previous_ms);
    imu_fusion_input_dt_ms = dt_ms;
    imu_fusion_reject_flags = trusted == 0u ? IMU_REJECT_TRUST : 0u;
    if (!(accel_pitch_deg >= -45.0f && accel_pitch_deg <= 45.0f)) {
        imu_fusion_reject_flags |= IMU_REJECT_PITCH;
    }
    if (!(rate_dps >= -IMU_GYRO_MAX_DPS && rate_dps <= IMU_GYRO_MAX_DPS)) {
        imu_fusion_reject_flags |= IMU_REJECT_RATE;
    }
    if (imu_fusion_reject_flags != 0u) {
        imu_fusion_invalidate();
        return;
    }
    if (imu_fusion_valid == 0u || dt_ms > 50u) {
        /* 首帧/长间隔后用重力角重新起步，不跨缺失时间盲目积分。 */
        imu_pitch_fused_deg = accel_pitch_deg;
        imu_fusion_dt_ms = 0u;
        ++imu_fusion_seed_count;
    } else {
        if (dt_ms == 0u) {
            imu_fusion_reject_flags = IMU_REJECT_ZERO_DT;
            imu_fusion_invalidate();
            return;
        }
        dt = (float)dt_ms * 0.001f;
        /* 互补滤波：陀螺给短时变化，加速度缓慢约束长期漂移。 */
        alpha = 0.5f / (0.5f + dt);  /* 时间常数0.5秒，不固定假设10ms。 */
        predicted = imu_pitch_fused_deg + rate_dps * dt;
        imu_pitch_fused_deg = alpha * predicted + (1.0f - alpha) * accel_pitch_deg;
        imu_fusion_dt_ms = dt_ms;
    }
    /* 只平移显示/控制参考，不改融合输入门限和角速度方向。 */
    imu_body_pitch_deg = imu_pitch_fused_deg - imu_upright_reference_deg;
    fusion_previous_ms = timestamp_ms;
#ifdef PD_GROUND_TEST
    fusion_trusted_ms = timestamp_ms; fusion_has_history = 1u;
#endif
    imu_gyro_only_age_ms = 0u; imu_fusion_mode = IMU_FUSION_FUSED;
    imu_fusion_valid = 1u;
}

#ifdef PD_GROUND_TEST
/* 仅模长失真时短暂舍弃重力校正；不重新定姿，不延长宽限。 */
static void gyro_only_update(float rate_dps, uint32_t stamp, uint32_t rejected)
{
    uint32_t dt_ms = (uint32_t)(stamp - fusion_previous_ms);
    uint32_t age = (uint32_t)(stamp - fusion_trusted_ms);
#ifndef PD_HOLD_TEST
    uint32_t limit = IMU_GYRO_ONLY_MAX_MS;
#endif
    float predicted;
    imu_fusion_input_dt_ms = dt_ms;
    imu_fusion_reject_flags = rejected;
    imu_gyro_only_age_ms = fusion_has_history != 0u ? age : 0u;
    if (imu_fusion_valid == 0u || fusion_has_history == 0u) {
        imu_fusion_reject_flags |= IMU_REJECT_GYRO_HISTORY;
        imu_fusion_invalidate(); return;
    }
    if (dt_ms == 0u || dt_ms > IMU_GYRO_ONLY_MAX_DT_MS) {
        imu_fusion_reject_flags |= IMU_REJECT_GYRO_DT;
        if (dt_ms == 0u) { imu_fusion_reject_flags |= IMU_REJECT_ZERO_DT; }
        imu_fusion_invalidate(); return;
    }
#ifndef PD_HOLD_TEST
    if (age > limit) {
        imu_fusion_reject_flags |= IMU_REJECT_GYRO_TIMEOUT;
        imu_fusion_invalidate(); return;
    }
#endif
    predicted = imu_pitch_fused_deg + rate_dps * ((float)dt_ms * 0.001f);
    if (!(predicted >= -45.0f && predicted <= 45.0f)) {
        imu_fusion_reject_flags |= IMU_REJECT_PITCH;
        imu_fusion_invalidate(); return;
    }
    imu_pitch_fused_deg = predicted;
    imu_body_pitch_deg = predicted - imu_upright_reference_deg;
    fusion_previous_ms = stamp;
    imu_fusion_dt_ms = dt_ms;
    imu_fusion_mode = IMU_FUSION_GYRO_ONLY;
    ++imu_gyro_only_count;
    if (age > imu_gyro_only_max_age_ms) { imu_gyro_only_max_age_ms = age; }
    imu_fusion_valid = 1u;
}
#endif
int32_t imu_decode_s16(uint8_t high, uint8_t low)
{
    uint32_t value = ((uint32_t)high << 8) | low;
    return value < 0x8000u ? (int32_t)value : (int32_t)value - 65536;
}

void imu_accel_angles(float ax, float ay, float az,
                      float *roll, float *pitch, uint32_t *valid)
{
    float norm2 = ax * ax + ay * ay + az * az;
    /* Static gravity tilt only; not a fused vehicle attitude estimate. */
    if (!(norm2 >= 0.25f && norm2 <= 2.25f)) {
        *roll = 0.0f;
        *pitch = 0.0f;
        *valid = 0u;
        return;
    }
    *roll = atan2f(ay, az) * 57.2957795f;
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;
    *valid = 1u;
}

void imu_publish_frame(const uint8_t frame[14], uint32_t timestamp_ms)
{
    float roll;
    float pitch;
    uint32_t angle_valid;
    uint32_t fusion_trusted, rejected;
    float norm2;

    ++imu_seq;                      /* Odd: publication in progress. */
    imu_ax_raw = imu_decode_s16(frame[0], frame[1]);
    imu_ay_raw = imu_decode_s16(frame[2], frame[3]);
    imu_az_raw = imu_decode_s16(frame[4], frame[5]);
    imu_temperature_raw = imu_decode_s16(frame[6], frame[7]);
    imu_gx_raw = imu_decode_s16(frame[8], frame[9]);
    imu_gy_raw = imu_decode_s16(frame[10], frame[11]);
    imu_gz_raw = imu_decode_s16(frame[12], frame[13]);
    imu_ax_g = (float)imu_ax_raw / 16384.0f; /* ACCEL_CONFIG=0: +/-2g. */
    imu_ay_g = (float)imu_ay_raw / 16384.0f;
    imu_az_g = (float)imu_az_raw / 16384.0f;
    imu_gx_dps = (float)imu_gx_raw / IMU_GYRO_LSB_PER_DPS; /* Matches the verified GYRO_CONFIG register. */
    imu_gy_dps = (float)imu_gy_raw / IMU_GYRO_LSB_PER_DPS;
    imu_gz_dps = (float)imu_gz_raw / IMU_GYRO_LSB_PER_DPS;
    imu_temperature_c = (float)imu_temperature_raw / 340.0f + 36.53f;
    imu_accel_angles(imu_ax_g, imu_ay_g, imu_az_g, &roll, &pitch, &angle_valid);
    imu_roll_deg = roll;
    imu_pitch_deg = pitch;
    imu_accel_angle_valid = angle_valid;
    imu_gx_corrected_dps = imu_gx_dps - imu_gyro_bias_x_dps;
    imu_gy_corrected_dps = imu_gy_dps - imu_gyro_bias_y_dps;
    imu_gz_corrected_dps = imu_gz_dps - imu_gyro_bias_z_dps;
    /* 抬车头时pitch增大、模块gy为负；先扣模块偏差，再取反。 */
    imu_pitch_rate_dps = -imu_gy_corrected_dps;
    norm2 = imu_ax_g * imu_ax_g + imu_ay_g * imu_ay_g + imu_az_g * imu_az_g;
    fusion_trusted = angle_valid != 0u && norm2 >= 0.7225f && norm2 <= 1.3225f &&
                     imu_az_g < -0.5f && imu_ay_g >= -IMU_LATERAL_MAX_G && imu_ay_g <= IMU_LATERAL_MAX_G;
    /* 模长0.85~1.15g且倒置近水平才融合；不是完整三轴姿态算法。 */
    imu_accel_norm2_g2 = norm2;
    rejected = fusion_trusted != 0u ? 0u : IMU_REJECT_TRUST;
    if (!(norm2 >= 0.7225f && norm2 <= 1.3225f)) { rejected |= IMU_REJECT_NORM; }
    if (!(imu_az_g < -0.5f)) { rejected |= IMU_REJECT_AXIS; }
    if (!(imu_ay_g >= -IMU_LATERAL_MAX_G && imu_ay_g <= IMU_LATERAL_MAX_G)) { rejected |= IMU_REJECT_LATERAL; }
    if (!(pitch >= -45.0f && pitch <= 45.0f)) { rejected |= IMU_REJECT_PITCH; }
    if (!(imu_pitch_rate_dps >= -IMU_GYRO_MAX_DPS && imu_pitch_rate_dps <= IMU_GYRO_MAX_DPS)) { rejected |= IMU_REJECT_RATE; }
#ifdef PD_GROUND_TEST
#ifdef PD_HOLD_TEST
    /* Acceleration gates select correction, never stop a valid gyro stream.
       The running window remains finite and body tilt is guarded downstream. */
    if (rejected != 0u && (rejected & IMU_REJECT_RATE) == 0u) {
#else
    if (angle_valid != 0u && rejected == (IMU_REJECT_TRUST | IMU_REJECT_NORM)) {
#endif
        gyro_only_update(imu_pitch_rate_dps, timestamp_ms, rejected);
    } else
#endif
    {
        imu_pitch_filter_update(pitch, imu_pitch_rate_dps, fusion_trusted, timestamp_ms);
        imu_fusion_reject_flags |= rejected;
    }
    imu_last_sample_ms = timestamp_ms;
    ++imu_sample_count;
    imu_valid = 1u;
    ++imu_seq;                      /* Even: complete sample. */
}

static void clear_observations(void)
{
    imu_state = IMU_WAITING;
    imu_who_am_i = 0u;
    imu_error = IMU_OK;
    imu_error_count = 0u;
    imu_i2c_stage = 0u;
    imu_sample_count = 0u;
    imu_last_sample_ms = 0u;
    imu_valid = 0u;
    imu_seq = 0u;
    imu_freeze = 0u;
    imu_frozen = 0u;
    imu_ax_raw = 0; imu_ay_raw = 0; imu_az_raw = 0;
    imu_gx_raw = 0; imu_gy_raw = 0; imu_gz_raw = 0;
    imu_temperature_raw = 0;
    imu_ax_g = 0.0f; imu_ay_g = 0.0f; imu_az_g = 0.0f;
    imu_gx_dps = 0.0f; imu_gy_dps = 0.0f; imu_gz_dps = 0.0f;
    imu_temperature_c = 0.0f;
    imu_roll_deg = 0.0f; imu_pitch_deg = 0.0f;
    imu_accel_angle_valid = 0u;
    imu_gx_corrected_dps = 0.0f; imu_gy_corrected_dps = 0.0f; imu_gz_corrected_dps = 0.0f;
    imu_pitch_rate_dps = 0.0f; imu_pitch_fused_deg = 0.0f; imu_body_pitch_deg = 0.0f;
    imu_fusion_invalidate();
    imu_fusion_seed_count = 0u; fusion_previous_ms = 0u;
    imu_fusion_reject_flags = IMU_REJECT_WAITING;
    imu_fusion_input_dt_ms = 0u; imu_accel_norm2_g2 = 0.0f;
#ifdef PD_GROUND_TEST
    fusion_trusted_ms = 0u; fusion_has_history = 0u;
#endif
    imu_fusion_mode = IMU_FUSION_INVALID; imu_gyro_only_age_ms = 0u;
    imu_gyro_only_count = 0u; imu_gyro_only_max_age_ms = 0u;
}

#if defined(IMU_READONLY) || defined(MOTOR_IMU_GUARD)

#define IMU_ADDRESS_WRITE 0xD0u
#define IMU_ADDRESS_READ  0xD1u
#define I2C_ERRORS (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR)
static uint32_t action_ms;

static int wait_sr1(uint32_t mask, uint32_t stage)
{
    uint32_t start = uptime_ms;
    uint32_t budget = 20000u;
    uint32_t status;
    imu_i2c_stage = stage;
    do {
        status = I2C1->SR1;
        if ((status & I2C_ERRORS) != 0u) {
            imu_error = (status & I2C_SR1_AF) != 0u ? IMU_NACK : IMU_BUS_ERROR;
            return 0;
        }
        if ((status & mask) == mask) { return 1; }
    } while (--budget != 0u && (uint32_t)(uptime_ms - start) < 5u);
    imu_error = IMU_TIMEOUT;
    return 0;
}

static int wait_control_clear(uint32_t mask, uint32_t stage)
{
    uint32_t start = uptime_ms;
    uint32_t budget = 20000u;
    imu_i2c_stage = stage;
    while ((I2C1->CR1 & mask) != 0u) {
        if (--budget == 0u || (uint32_t)(uptime_ms - start) >= 5u) {
            imu_error = IMU_TIMEOUT;
            return 0;
        }
    }
    return 1;
}

static void clear_address(void)
{
    volatile uint32_t dummy;
    dummy = I2C1->SR1;
    dummy = I2C1->SR2;
    (void)dummy;
}

static int start_write(uint32_t reg)
{
    uint32_t start = uptime_ms;
    uint32_t budget = 20000u;
    imu_i2c_stage = 1u;
    while ((I2C1->SR2 & I2C_SR2_BUSY) != 0u) {
        if (--budget == 0u || (uint32_t)(uptime_ms - start) >= 5u) {
            imu_error = IMU_BUS_BUSY;
            return 0;
        }
    }
    I2C1->CR1 = I2C_CR1_PE | I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_sr1(I2C_SR1_SB, 2u)) { return 0; }
    I2C1->DR = IMU_ADDRESS_WRITE;
    if (!wait_sr1(I2C_SR1_ADDR, 3u)) { return 0; }
    clear_address();
    if (!wait_sr1(I2C_SR1_TXE, 4u)) { return 0; }
    I2C1->DR = reg;
    return wait_sr1(I2C_SR1_BTF, 5u);
}

static int write_register(uint32_t reg, uint32_t value)
{
    if (!start_write(reg)) { return 0; }
    I2C1->DR = value;
    if (!wait_sr1(I2C_SR1_BTF, 6u)) { return 0; }
    I2C1->CR1 |= I2C_CR1_STOP;
    return wait_control_clear(I2C_CR1_STOP, 7u);
}

static int read_registers(uint32_t reg, uint8_t *data, uint32_t length)
{
    uint32_t saved_primask;
    if (length != 1u && length < 3u) { return 0; }
    if (!start_write(reg)) { return 0; }
    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_sr1(I2C_SR1_SB, 8u)) { return 0; }
    I2C1->DR = IMU_ADDRESS_READ;
    if (!wait_sr1(I2C_SR1_ADDR, 9u)) { return 0; }

    if (length == 1u) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        saved_primask = __get_PRIMASK();
        __disable_irq();
        clear_address();
        I2C1->CR1 |= I2C_CR1_STOP;
        __set_PRIMASK(saved_primask);
        if (!wait_sr1(I2C_SR1_RXNE, 10u)) { return 0; }
        *data = (uint8_t)I2C1->DR;
    } else {
        clear_address();
        /* BTF stretches SCL until DR is read; robust at the 8 MHz CPU clock. */
        while (length > 3u) {
            if (!wait_sr1(I2C_SR1_BTF, 11u)) { return 0; }
            *data++ = (uint8_t)I2C1->DR;
            --length;
        }
        if (!wait_sr1(I2C_SR1_BTF, 12u)) { return 0; }
        I2C1->CR1 &= ~I2C_CR1_ACK;
        saved_primask = __get_PRIMASK();
        __disable_irq();
        *data++ = (uint8_t)I2C1->DR;  /* N-2: begin reception of final byte. */
        __set_PRIMASK(saved_primask);
        if (!wait_sr1(I2C_SR1_BTF, 13u)) { return 0; }
        saved_primask = __get_PRIMASK();
        __disable_irq();
        I2C1->CR1 |= I2C_CR1_STOP;
        *data++ = (uint8_t)I2C1->DR;
        *data = (uint8_t)I2C1->DR;
        __set_PRIMASK(saved_primask);
    }
    if (!wait_control_clear(I2C_CR1_STOP, 14u)) { return 0; }
    I2C1->CR1 |= I2C_CR1_ACK;
    return 1;
}

static void bus_delay(void)
{
    volatile uint32_t n;
    for (n = 0u; n < 32u; ++n) { __NOP(); }
}

static void configure_i2c(void)
{
    uint32_t n;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->CR1 = 0u;
    /* Temporarily use open-drain GPIO to release a previously stuck slave. */
    GPIOB->BSRR = (1u << 8) | (1u << 9);
    GPIOB->CRH = (GPIOB->CRH & ~0xFFu) | 0x66u;
    bus_delay();
    for (n = 0u; n < 9u && (GPIOB->IDR & (1u << 9)) == 0u; ++n) {
        GPIOB->BRR = 1u << 8;
        bus_delay();
        GPIOB->BSRR = 1u << 8;
        bus_delay();
    }
    /* Generate STOP only when SCL is actually released high. */
    if ((GPIOB->IDR & (1u << 8)) != 0u) {
        GPIOB->BRR = 1u << 9;
        bus_delay();
        GPIOB->BSRR = 1u << 9;
        bus_delay();
    }
    AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;
    GPIOB->CRH = (GPIOB->CRH & ~0xFFu) | 0xEEu; /* PB8/9 AF open drain. */
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    I2C1->CR2 = 8u;                 /* APB1=8 MHz. */
    I2C1->OAR1 = 0x4000u;
    I2C1->CCR = 40u;                /* 8 MHz/(2*40)=100 kHz. */
    I2C1->TRISE = 9u;
    I2C1->CR1 = I2C_CR1_PE | I2C_CR1_ACK;
}

static void record_failure(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->SR1 &= ~I2C_ERRORS;
    I2C1->CR1 &= ~I2C_CR1_PE;
    imu_state = IMU_FAULT;
    imu_valid = 0u;
    imu_accel_angle_valid = 0u;
    imu_fusion_reject_flags = IMU_REJECT_BUS;
    imu_fusion_invalidate();
    ++imu_error_count;
    action_ms = uptime_ms;
}

void imu_init(void)
{
    clear_observations();
    action_ms = uptime_ms;
    configure_i2c();
}

void imu_poll(void)
{
    uint8_t identity;
    uint8_t frame[15];
    uint32_t now = uptime_ms;

    if (imu_freeze != 0u) {
        imu_frozen = 1u;            /* Debug snapshot, not a live sample. */
        return;
    }
    imu_frozen = 0u;
    if (imu_valid != 0u && (uint32_t)(now - imu_last_sample_ms) > 100u) {
        imu_valid = 0u;
        imu_accel_angle_valid = 0u;
        imu_fusion_reject_flags = IMU_REJECT_STALE;
        imu_fusion_invalidate();
    }
    if (imu_state == IMU_FAULT) {
        if ((uint32_t)(now - action_ms) < 1000u) { return; }
        configure_i2c();
        imu_state = IMU_WAITING;
        action_ms = now;
        return;
    }
    if (imu_state == IMU_WAITING) {
        if ((uint32_t)(now - action_ms) < 100u) { return; }
        imu_error = IMU_OK;
        if (!read_registers(0x75u, &identity, 1u)) { record_failure(); return; }
        imu_who_am_i = identity;
        if (identity != 0x68u) {
            imu_error = IMU_ID_MISMATCH;
            record_failure();
            return;
        }
        if (!write_register(0x6Bu, 0x80u)) { record_failure(); return; }
        imu_state = IMU_CONFIGURING;
        action_ms = uptime_ms;
        return;
    }
    if (imu_state == IMU_CONFIGURING) {
        if ((uint32_t)(now - action_ms) < 100u) { return; }
        if (!write_register(0x6Bu, 1u) || !write_register(0x6Cu, 0u) ||
            !write_register(0x19u, 9u) || !write_register(0x1Au, 3u) ||
            !write_register(0x1Bu, IMU_GYRO_CONFIG_VALUE) || !write_register(0x1Cu, 0u) ||
            !write_register(0x38u, 1u)) { record_failure(); return; }
        if (!read_registers(0x1Bu, &identity, 1u)) { record_failure(); return; }
        imu_gyro_config_readback = identity;
        if (identity != IMU_GYRO_CONFIG_VALUE) {
            imu_error = IMU_BUS_ERROR; record_failure(); return;
        }
        imu_state = IMU_READING;
        action_ms = uptime_ms;
        return;
    }
    if ((uint32_t)(now - action_ms) < 5u) { return; }
    action_ms = now;
    /* Burst INT_STATUS plus 14 contiguous bytes; publish fresh samples only. */
    if (!read_registers(0x3Au, frame, 15u)) { record_failure(); return; }
    if ((frame[0] & 1u) != 0u) {
        imu_publish_frame(&frame[1], uptime_ms);
    }
}

#else
/* Existing motor and simulator targets do not access the real IMU bus. */
void imu_init(void) { clear_observations(); }
void imu_poll(void) { }
#endif

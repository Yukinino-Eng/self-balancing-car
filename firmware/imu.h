#ifndef IMU_H
#define IMU_H

#include "board.h"

enum { IMU_WAITING = 0, IMU_CONFIGURING = 1, IMU_READING = 2, IMU_FAULT = 3 };
enum { IMU_OK = 0, IMU_BUS_BUSY = 1, IMU_NACK = 2,
       IMU_TIMEOUT = 3, IMU_ID_MISMATCH = 4, IMU_BUS_ERROR = 5 };

extern volatile uint32_t imu_state;
extern volatile uint32_t imu_who_am_i;
extern volatile uint32_t imu_error;
extern volatile uint32_t imu_error_count;
extern volatile uint32_t imu_i2c_stage;
extern volatile uint32_t imu_sample_count;
extern volatile uint32_t imu_last_sample_ms;
extern volatile uint32_t imu_valid;
extern volatile uint32_t imu_seq;
extern volatile uint32_t imu_freeze;
extern volatile uint32_t imu_frozen;
extern volatile int32_t imu_ax_raw, imu_ay_raw, imu_az_raw;
extern volatile int32_t imu_gx_raw, imu_gy_raw, imu_gz_raw;
extern volatile int32_t imu_temperature_raw;
extern volatile float imu_ax_g, imu_ay_g, imu_az_g;
extern volatile float imu_gx_dps, imu_gy_dps, imu_gz_dps;
extern volatile float imu_temperature_c;
extern volatile float imu_roll_deg, imu_pitch_deg;
extern volatile uint32_t imu_accel_angle_valid;

extern const float imu_gyro_bias_x_dps, imu_gyro_bias_y_dps, imu_gyro_bias_z_dps;
extern volatile float imu_gx_corrected_dps, imu_gy_corrected_dps, imu_gz_corrected_dps;
extern volatile float imu_pitch_rate_dps, imu_pitch_fused_deg;
extern volatile uint32_t imu_fusion_valid, imu_fusion_dt_ms, imu_fusion_seed_count;
/* 车体相对直立倾角，仅在imu_fusion_valid=1时可用。 */
extern const volatile float imu_upright_reference_deg;
extern volatile float imu_body_pitch_deg;

/* 当前帧拒绝原因位图，只用于诊断，不改变融合门槛。 */
enum {
    IMU_REJECT_TRUST = 1u, IMU_REJECT_NORM = 2u,
    IMU_REJECT_AXIS = 4u, IMU_REJECT_LATERAL = 8u,
    IMU_REJECT_PITCH = 16u, IMU_REJECT_RATE = 32u,
    IMU_REJECT_ZERO_DT = 64u, IMU_REJECT_BUS = 128u,
    IMU_REJECT_STALE = 256u, IMU_REJECT_WAITING = 512u,
    IMU_REJECT_GYRO_TIMEOUT = 1024u, IMU_REJECT_GYRO_DT = 2048u,
    IMU_REJECT_GYRO_HISTORY = 4096u
};
extern volatile uint32_t imu_fusion_reject_flags, imu_fusion_input_dt_ms;
extern volatile float imu_accel_norm2_g2;
/* Hardware range, conversion and all software guards must agree. */
#ifdef PD_HOLD_TEST
#define IMU_GYRO_CONFIG_VALUE 0x08u
#define IMU_GYRO_LSB_PER_DPS 65.5f
#define IMU_GYRO_MAX_DPS 500.0f
#define IMU_LATERAL_MAX_G 0.60f
#else
#define IMU_GYRO_CONFIG_VALUE 0u
#define IMU_GYRO_LSB_PER_DPS 131.0f
#define IMU_GYRO_MAX_DPS 250.0f
#define IMU_LATERAL_MAX_G 0.35f
#endif
extern volatile uint32_t imu_gyro_config_readback;
#define IMU_GYRO_ONLY_MAX_MS 30u
#define IMU_GYRO_ONLY_MAX_DT_MS 20u
#ifdef PD_HOLD_TEST
#define IMU_GYRO_PREDICT_DEFAULT_MS 120u
#define IMU_GYRO_PREDICT_MAX_MS 300u
extern volatile uint32_t imu_gyro_predict_limit_ms;
#endif
enum { IMU_FUSION_INVALID=0, IMU_FUSION_FUSED=1, IMU_FUSION_GYRO_ONLY=2 };
extern const volatile uint32_t imu_gyro_only_enabled;
extern volatile uint32_t imu_fusion_mode, imu_gyro_only_age_ms;
extern volatile uint32_t imu_gyro_only_count, imu_gyro_only_max_age_ms;
void imu_fusion_invalidate(void);
void imu_pitch_filter_update(float accel_pitch_deg, float rate_dps,
                             uint32_t trusted, uint32_t timestamp_ms);
void imu_init(void);
void imu_poll(void);
int32_t imu_decode_s16(uint8_t high, uint8_t low);
void imu_accel_angles(float ax, float ay, float az,
                      float *roll, float *pitch, uint32_t *valid);
void imu_publish_frame(const uint8_t frame[14], uint32_t timestamp_ms);

#endif

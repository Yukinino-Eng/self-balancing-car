#include "hold_tuning.h"
#ifdef PD_HOLD_TEST
#include "hold_config.h"
#include "board.h"
#include "balance_preview.h"
#include "pd_window.h"
#include "imu.h"
#include "speed_loop.h"
#include "param_store.h"
#include "local_control.h"
#include "task_motion.h"
const volatile uint32_t tune_api_version = 7u;
volatile float tune_requested_task_speed=TASK_DEFAULT_SPEED_CMS;
volatile uint32_t tune_requested_task_ramp_ms=TASK_DEFAULT_RAMP_MS;
volatile uint32_t hold_common_boost_counts=HOLD_COMMON_BOOST_COUNTS;
volatile float hold_common_boost_percent=(float)HOLD_COMMON_BOOST_COUNTS*0.25f;
volatile uint32_t tune_requested_common_boost_counts=HOLD_COMMON_BOOST_COUNTS;
volatile float tune_requested_kp = 4.0f, tune_requested_kd = 0.05f;
volatile float tune_requested_speed_kp=SPEED_DEFAULT_KP, tune_requested_speed_ki=SPEED_DEFAULT_KI;
volatile float tune_requested_speed_tilt=SPEED_DEFAULT_TILT_DEG;
volatile float tune_requested_backward_kp = HOLD_BACKWARD_DEFAULT_KP;
volatile uint32_t tune_requested_backward_limit = HOLD_DEFAULT_BACKWARD_LIMIT_PERCENT;
volatile uint32_t tune_requested_limit = HOLD_DEFAULT_FORWARD_LIMIT_PERCENT;
volatile uint32_t tune_requested_gyro_predict_ms = IMU_GYRO_PREDICT_DEFAULT_MS;
volatile uint32_t tune_requested_window_ms = 6000u;
volatile uint32_t tune_requested_step_ms = HOLD_START_STEP_MS;
volatile uint32_t tune_requested_boost_counts = HOLD_RIGHT_BOOST_COUNTS;
volatile uint32_t tune_requested_boost_base = HOLD_RIGHT_BOOST_MAX_BASE_PERCENT;
volatile uint32_t tune_command, tune_status, tune_error_field;
volatile uint32_t tune_request_id, tune_result_id, tune_revision;

static uint32_t invalid_field(float kp, float kd, uint32_t limit,
    uint32_t window, uint32_t step, uint32_t boost, uint32_t base,
    uint32_t backward_limit, float backward_kp, uint32_t gyro_predict_ms,
    float vkp, float vki, float vtilt, uint32_t common, float task_speed,uint32_t task_ramp)
{
    uint32_t max_limit = limit > backward_limit ? limit : backward_limit;
    if (!(kp >= 0.0f && kp <= HOLD_MAX_KP)) { return 1u; }
    if (!(kd >= 0.0f && kd <= HOLD_MAX_KD)) { return 2u; }
    if (limit == 0u || limit > HOLD_OUTPUT_LIMIT_PERCENT) { return 3u; }
    if (backward_limit == 0u || backward_limit > HOLD_BACKWARD_LIMIT_PERCENT) { return 8u; }
    if (!(backward_kp >= 0.0f && backward_kp <= HOLD_MAX_KP)) { return 9u; }
    if (gyro_predict_ms < IMU_GYRO_ONLY_MAX_MS || gyro_predict_ms > IMU_GYRO_PREDICT_MAX_MS) { return 10u; }
    if (window < 100u || window > TUNE_MAX_WINDOW_MS) { return 4u; }
    if (step > TUNE_MAX_STEP_MS || (max_limit-1u)*step > window) { return 5u; }
    if (boost > TUNE_MAX_BOOST_COUNTS) { return 6u; }
    if (base == 0u || base > HOLD_OUTPUT_LIMIT_PERCENT) { return 7u; }
    if (!(vkp>=0.0f && vkp<=SPEED_MAX_KP)) return 11u;
    if (!(vki>=0.0f && vki<=SPEED_MAX_KI)) return 12u;
    if (!(vtilt>=0.0f && vtilt<=SPEED_MAX_TILT_DEG)) return 13u;
    if (common>HOLD_MAX_COMMON_BOOST_COUNTS) return 14u;
    if (!(task_speed>=0.5f && task_speed<=10.0f))return 15u;
    if (task_ramp<300u || task_ramp>2000u)return 16u;
    return 0u;
}

uint32_t hold_tuning_active_valid(void)
{
    return invalid_field(balance_kp, balance_kd, hold_output_limit_percent,
        pd_window_limit_ms, hold_start_step_ms, hold_right_boost_counts,
        hold_right_boost_max_base_percent, hold_backward_limit_percent,
        balance_backward_kp, imu_gyro_predict_limit_ms, speed_kp, speed_ki, speed_tilt_limit_deg,
        hold_common_boost_counts,task_speed_cms,task_ramp_ms) == 0u;
}

void hold_tuning_init(void)
{
    task_speed_cms=TASK_DEFAULT_SPEED_CMS;task_ramp_ms=TASK_DEFAULT_RAMP_MS;
    tune_requested_task_speed=task_speed_cms;tune_requested_task_ramp_ms=task_ramp_ms;
    hold_common_boost_counts=HOLD_COMMON_BOOST_COUNTS;
    hold_common_boost_percent=(float)hold_common_boost_counts*0.25f;
    tune_requested_common_boost_counts=hold_common_boost_counts;
    hold_output_limit_percent = HOLD_DEFAULT_FORWARD_LIMIT_PERCENT;
    hold_backward_limit_percent = HOLD_DEFAULT_BACKWARD_LIMIT_PERCENT;
    pd_window_limit_ms = PD_WINDOW_RUN_MS;
    imu_gyro_predict_limit_ms = IMU_GYRO_PREDICT_DEFAULT_MS;
    hold_start_step_ms = HOLD_START_STEP_MS;
    hold_start_full_ms = HOLD_START_FULL_MS;
    hold_backward_start_full_ms = (HOLD_DEFAULT_BACKWARD_LIMIT_PERCENT-1u)*HOLD_START_STEP_MS;
    hold_right_boost_counts = HOLD_RIGHT_BOOST_COUNTS;
    hold_right_boost_percent = (float)HOLD_RIGHT_BOOST_COUNTS * 0.25f;
    hold_right_boost_max_base_percent = HOLD_RIGHT_BOOST_MAX_BASE_PERCENT;
    tune_requested_speed_kp=speed_kp; tune_requested_speed_ki=speed_ki;
    tune_requested_speed_tilt=speed_tilt_limit_deg;
    tune_requested_kp = balance_kp; tune_requested_kd = balance_kd;
    tune_requested_backward_kp = balance_backward_kp;
    tune_requested_backward_limit = hold_backward_limit_percent;
    tune_requested_gyro_predict_ms = imu_gyro_predict_limit_ms;
    tune_requested_limit = hold_output_limit_percent;
    tune_requested_window_ms = pd_window_limit_ms;
    tune_requested_step_ms = hold_start_step_ms;
    tune_requested_boost_counts = hold_right_boost_counts;
    tune_requested_boost_base = hold_right_boost_max_base_percent;
    tune_command = 0u; tune_status = TUNE_IDLE; tune_error_field = 0u;
    tune_request_id = 0u; tune_result_id = 0u; tune_revision = 0u;
}

/* Called under pd_window_poll's IRQ lock. Commit all fields or none.
   Applying parameters never starts, stops, clears faults, or extends a run. */
void hold_tuning_poll(void)
{
    uint32_t command = tune_command, field, request;
    float kp, kd, backward_kp, vkp, vki, vtilt, task_speed;
    uint32_t limit, window, step, boost, base, backward_limit, gyro_predict_ms, common, task_ramp;
    if (command == 0u) { return; }
    tune_command = 0u; request = tune_request_id;
    tune_error_field = 0u;
    if (pd_window_active != 0u || local_state==LOCAL_PREPARING || local_state==LOCAL_REQUESTED) { tune_status = TUNE_BUSY; }
    else if (command != 1u) { tune_status = TUNE_INVALID; }
    else {
        vkp=tune_requested_speed_kp; vki=tune_requested_speed_ki;
        vtilt=tune_requested_speed_tilt;
        kp = tune_requested_kp; kd = tune_requested_kd;
        backward_kp = tune_requested_backward_kp;
        backward_limit = tune_requested_backward_limit;
        gyro_predict_ms = tune_requested_gyro_predict_ms;
        limit = tune_requested_limit; window = tune_requested_window_ms;
        step = tune_requested_step_ms; boost = tune_requested_boost_counts;
        base = tune_requested_boost_base;
        common=tune_requested_common_boost_counts;
        task_speed=tune_requested_task_speed;task_ramp=tune_requested_task_ramp_ms;
        field = invalid_field(kp, kd, limit, window, step, boost, base,
                              backward_limit, backward_kp, gyro_predict_ms, vkp, vki, vtilt, common,task_speed,task_ramp);
        if (field != 0u) { tune_error_field = field; tune_status = TUNE_INVALID; }
        else {
            task_speed_cms=task_speed;task_ramp_ms=task_ramp;
            hold_common_boost_counts=common;hold_common_boost_percent=(float)common*0.25f;
            speed_kp=vkp; speed_ki=vki; speed_tilt_limit_deg=vtilt; speed_loop_reset();
            balance_kp = kp; balance_kd = kd;
            balance_backward_kp = backward_kp;
            hold_backward_limit_percent = backward_limit;
            imu_gyro_predict_limit_ms = gyro_predict_ms;
            hold_output_limit_percent = limit; pd_window_limit_ms = window;
            hold_start_step_ms = step; hold_start_full_ms = (limit-1u)*step;
            hold_backward_start_full_ms = (backward_limit-1u)*step;
            hold_right_boost_counts = boost;
            hold_right_boost_percent = (float)boost * 0.25f;
            hold_right_boost_max_base_percent = base;
            ++tune_revision; tune_status = TUNE_APPLIED; param_store_mark_dirty();
        }
    }
    tune_result_id = request; /* Publish acknowledgement last. */
}
static uint32_t float_bits(float f){union{float f;uint32_t u;}v;v.f=f;return v.u;}
static float bits_float(uint32_t u){union{float f;uint32_t u;}v;v.u=u;return v.f;}
void hold_tuning_export_record(uint32_t d[16])
{
    d[0]=float_bits(balance_kp);d[1]=float_bits(balance_backward_kp);d[2]=float_bits(balance_kd);
    d[3]=hold_output_limit_percent;d[4]=hold_backward_limit_percent;d[5]=pd_window_limit_ms;
    d[6]=hold_start_step_ms;d[7]=hold_right_boost_counts;d[8]=hold_right_boost_max_base_percent;
    d[9]=imu_gyro_predict_limit_ms;d[10]=float_bits(speed_kp);d[11]=float_bits(speed_ki);d[12]=float_bits(speed_tilt_limit_deg);
    d[13]=hold_common_boost_counts;
    d[14]=float_bits(task_speed_cms);d[15]=task_ramp_ms;
}
uint32_t hold_tuning_record_valid(const uint32_t d[16])
{
    return invalid_field(bits_float(d[0]),bits_float(d[2]),d[3],d[5],d[6],d[7],d[8],d[4],bits_float(d[1]),d[9],bits_float(d[10]),bits_float(d[11]),bits_float(d[12]),d[13],bits_float(d[14]),d[15])==0u;
}
uint32_t hold_tuning_load_record(const uint32_t d[16])
{
    if(pd_window_active || !hold_tuning_record_valid(d))return 0u;
    tune_requested_kp=bits_float(d[0]);tune_requested_backward_kp=bits_float(d[1]);tune_requested_kd=bits_float(d[2]);
    tune_requested_limit=d[3];tune_requested_backward_limit=d[4];tune_requested_window_ms=d[5];
    tune_requested_step_ms=d[6];tune_requested_boost_counts=d[7];tune_requested_boost_base=d[8];
    tune_requested_gyro_predict_ms=d[9];tune_requested_speed_kp=bits_float(d[10]);tune_requested_speed_ki=bits_float(d[11]);tune_requested_speed_tilt=bits_float(d[12]);
    tune_requested_common_boost_counts=d[13];
    tune_requested_task_speed=bits_float(d[14]);tune_requested_task_ramp_ms=d[15];
    ++tune_request_id;tune_command=1u;hold_tuning_poll();return tune_status==TUNE_APPLIED;
}
#endif

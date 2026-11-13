#include "speed_loop.h"
#ifdef PD_HOLD_TEST
#include "encoder.h"
#include "runtime.h"
#include "pd_window.h"
#include "balance_preview.h"
#include "imu.h"
#include "board.h"
#include "task_motion.h"

volatile float speed_kp=SPEED_DEFAULT_KP, speed_ki=SPEED_DEFAULT_KI;
volatile float speed_tilt_limit_deg=SPEED_DEFAULT_TILT_DEG;
volatile float speed_left_cps, speed_right_cps, speed_filtered_cps;
volatile float speed_integral_deg, speed_target_body_deg;
volatile float speed_error_cps;
volatile float speed_wheel_difference_cps, speed_wheel_correction_percent;
volatile float speed_wheel_integral_percent;
volatile float speed_wheel_motion_blend;
volatile uint32_t speed_update_count, speed_state;
static uint32_t previous_left, previous_right, previous_ms;
#ifndef PARALLEL_PID_TRIAL
static uint32_t sync_tracking,sync_run;
static float sync_previous_error_cm;
#endif
#ifdef PARALLEL_PID_TRIAL
#include "parallel_trial_config.h"
volatile float speed_parallel_kp=PAR_TRIAL_SPEED_KP,speed_parallel_ki=PAR_TRIAL_SPEED_KI;
volatile float speed_parallel_limit_percent=PAR_TRIAL_SPEED_LIMIT;
volatile float speed_parallel_p_percent,speed_parallel_i_percent,speed_parallel_output_percent;
static float parallel_dt;
const volatile uint32_t position_hold_api=2u;
volatile uint32_t position_hold_state;
volatile float position_hold_offset_cm,position_hold_target_cms,speed_effective_reference_cps;
static wheel_origin_t position_hold_origin;
static uint32_t position_hold_stable_ms,position_hold_tracking;
static uint32_t position_hold_anchor_valid,position_hold_immediate;
static float position_hold_anchor_bias;
#endif

static float clip(float value, float limit)
{
    return value>limit ? limit : value < -limit ? -limit : value;
}
uint32_t speed_loop_config_valid(float kp,float ki,float tilt)
{
    return kp>=0.0f && kp<=SPEED_MAX_KP && ki>=0.0f && ki<=SPEED_MAX_KI &&
           tilt>=0.0f && tilt<=SPEED_MAX_TILT_DEG;
}
#ifdef PARALLEL_PID_TRIAL
uint32_t speed_loop_parallel_config_valid(float kp,float ki,float limit)
{
    return kp>=0.0f && kp<=100.0f && ki>=0.0f && ki<=50.0f &&
           limit>=0.0f && limit<=PAR_TRIAL_SPEED_MAX_LIMIT;
}
static float magnitude(float value){return value<0.0f ? -value : value;}
static void position_hold_reset(void)
{
    position_hold_state=POSITION_HOLD_OFF;position_hold_tracking=0u;
    position_hold_anchor_valid=0u;position_hold_immediate=0u;position_hold_anchor_bias=0.0f;
    position_hold_offset_cm=0.0f;position_hold_target_cms=0.0f;
    speed_effective_reference_cps=0.0f;
}
void speed_loop_position_hold_at(float chassis_error_cm)
{
    if(!pd_window_active || !speed_loop_command_enabled() ||
       !(chassis_error_cm>=-100.0f && chassis_error_cm<=100.0f))return;
    wheel_odometry_origin(&position_hold_origin,(uint32_t)encoder_e1_count,(uint32_t)encoder_e2_count);
    position_hold_anchor_bias=chassis_error_cm;position_hold_anchor_valid=1u;
    position_hold_immediate=1u;position_hold_tracking=0u;
    position_hold_offset_cm=chassis_error_cm;position_hold_target_cms=0.0f;
    position_hold_state=POSITION_HOLD_WAITING;
}
static void position_hold_poll(uint32_t left,uint32_t right,uint32_t now)
{
    float body=magnitude(imu_body_pitch_deg),rate=magnitude(imu_pitch_rate_dps);
    float speed=magnitude(speed_filtered_cps/TASK_COUNTS_PER_CM),amount;
    wheel_distance_t distance;
    uint32_t parking=task_state==TASK_BRAKING || task_state==TASK_SETTLING || task_state==TASK_FAILED ||
                     (task_state==TASK_DISTANCE && task_distance_holding);
    position_hold_target_cms=0.0f;
    if(!speed_loop_command_enabled()){position_hold_reset();return;}
    if(task_state!=TASK_BALANCE && task_state!=TASK_DONE && !parking){
        position_hold_state=POSITION_HOLD_TASK;position_hold_tracking=0u;
        position_hold_anchor_valid=0u;position_hold_immediate=0u;
        position_hold_offset_cm=0.0f;return;
    }
    if(body>=POSITION_HOLD_EXIT_DEG || rate>=POSITION_HOLD_EXIT_DPS || speed>=POSITION_HOLD_EXIT_CMS){
        position_hold_state=POSITION_HOLD_DISTURBED;position_hold_tracking=0u;
        position_hold_anchor_valid=0u;position_hold_immediate=0u;
        position_hold_offset_cm=0.0f;return;
    }
    if(!position_hold_anchor_valid){
        wheel_odometry_origin(&position_hold_origin,left,right);
        position_hold_anchor_bias=(task_state==TASK_DISTANCE && task_distance_holding)?task_distance_error_cm:0.0f;
        position_hold_anchor_valid=1u;
    }
    if(position_hold_state!=POSITION_HOLD_ACTIVE){
        if(position_hold_state!=POSITION_HOLD_DISTURBED)position_hold_state=POSITION_HOLD_WAITING;
        if(body>POSITION_HOLD_ENTER_DEG || rate>POSITION_HOLD_ENTER_DPS || speed>POSITION_HOLD_ENTER_CMS){
            position_hold_tracking=0u;return;
        }
        if(!position_hold_immediate && !parking){
            if(!position_hold_tracking){position_hold_stable_ms=now;position_hold_tracking=1u;return;}
            if((uint32_t)(now-position_hold_stable_ms)<POSITION_HOLD_STABLE_MS)return;
        }
        /* Retain the captured stop target instead of recentering after coast. */
        position_hold_state=POSITION_HOLD_ACTIVE;position_hold_tracking=0u;position_hold_immediate=0u;
    }
    wheel_odometry_measure(&position_hold_origin,left,right,&distance);
    position_hold_offset_cm=distance.mean_cm+position_hold_anchor_bias;
    amount=magnitude(position_hold_offset_cm)-POSITION_HOLD_FREE_CM;
    if(amount>0.0f){
        amount=clip(POSITION_HOLD_KP*amount,POSITION_HOLD_MAX_CMS);
        position_hold_target_cms=position_hold_offset_cm>0.0f ? -amount : amount;
    }
}
static void parallel_wheel_sync(float dt)
{
    float increment,candidate,p,request;
    uint32_t unwind;
    /* Integral of signed encoder wheel travel difference; independent of
       task origins, so starting or ending a task cannot erase the trim. */
    increment=PAR_WHEEL_SYNC_POSITION_GAIN*(speed_right_cps-speed_left_cps)*dt/TASK_COUNTS_PER_CM;
    candidate=clip(speed_wheel_integral_percent+increment,PAR_WHEEL_SYNC_I_LIMIT);
    p=PAR_WHEEL_SYNC_KP*speed_wheel_difference_cps*0.001f;
    request=p+candidate;
    unwind=(speed_wheel_integral_percent>0.0f && increment<0.0f) ||
           (speed_wheel_integral_percent<0.0f && increment>0.0f);
    if(unwind || (imu_body_pitch_deg>=-PAR_WHEEL_SYNC_I_TILT_DEG &&
       imu_body_pitch_deg<=PAR_WHEEL_SYNC_I_TILT_DEG && observed_stby &&
       !balance_output_limited && !hold_actuator_limited &&
       !((hold_sync_saturation_error_counts>0 && increment>0.0f) ||
         (hold_sync_saturation_error_counts<0 && increment<0.0f)) &&
       !((request>PAR_WHEEL_SYNC_LIMIT && increment>0.0f) ||
         (request< -PAR_WHEEL_SYNC_LIMIT && increment<0.0f))))
        speed_wheel_integral_percent=candidate;
    request=clip(p+speed_wheel_integral_percent,PAR_WHEEL_SYNC_LIMIT);
    speed_wheel_correction_percent+=clip(request-speed_wheel_correction_percent,
                                        PAR_WHEEL_SYNC_SLEW_PPS*dt);
    speed_wheel_motion_blend=1.0f; /* Whole-session sync; legacy diagnostic. */
}
static void parallel_reset(void)
{
    parallel_dt=0.0f;
    speed_parallel_p_percent=0.0f;speed_parallel_i_percent=0.0f;speed_parallel_output_percent=0.0f;
}
static float parallel_apply_locked(float angle_pwm,float forward_limit,float backward_limit)
{
    float dt=parallel_dt,limit=speed_parallel_limit_percent,p,increment,candidate,speed,sum;
    parallel_dt=0.0f;
    if(!pd_window_active || !speed_loop_command_enabled()) {parallel_reset();return 0.0f;}
    if(dt==0.0f)return speed_parallel_output_percent;
    p=speed_parallel_kp*speed_error_cps*0.001f;
    increment=speed_parallel_ki*speed_error_cps*0.001f*dt;
    candidate=clip(speed_parallel_i_percent+increment,limit);
    speed=p+candidate;
    sum=angle_pwm+clip(speed,limit);
    /* Inhibit only the direction that drives speed or final PWM farther into
       clipping. Use the live angle term plus the prior actual driver limit. */
    if(imu_body_pitch_deg>=-8.0f && imu_body_pitch_deg<=8.0f &&
       !((increment>0.0f && (speed>limit || sum>forward_limit ||
           (hold_actuator_limited && pd_window_applied_counts>0))) ||
         (increment<0.0f && (speed< -limit || sum< -backward_limit ||
           (hold_actuator_limited && pd_window_applied_counts<0)))))
        speed_parallel_i_percent=candidate;
    speed_parallel_p_percent=p;
    speed=clip(p+speed_parallel_i_percent,limit);
    /* Apply speed PI on this encoder update without an extra slew delay. */
    speed_parallel_output_percent=speed;
    return speed_parallel_output_percent;
}
float speed_loop_parallel_apply(float angle_pwm,float forward_limit,float backward_limit)
{
    uint32_t saved=__get_PRIMASK();float output;
    __disable_irq();
    output=parallel_apply_locked(angle_pwm,forward_limit,backward_limit);
    __set_PRIMASK(saved);
    return output;
}
#endif
uint32_t speed_loop_command_enabled(void)
{
#ifdef PARALLEL_PID_TRIAL
    return speed_loop_parallel_config_valid(speed_parallel_kp,speed_parallel_ki,speed_parallel_limit_percent) &&
           speed_parallel_limit_percent>0.0f &&
           (speed_parallel_kp!=0.0f || speed_parallel_ki!=0.0f);
#else
    return speed_loop_config_valid(speed_kp,speed_ki,speed_tilt_limit_deg) &&
           speed_tilt_limit_deg>0.0f && (speed_kp!=0.0f || speed_ki!=0.0f);
#endif
}
void speed_loop_reset(void)
{
    uint32_t saved=__get_PRIMASK();
    __disable_irq();
    previous_left=(uint32_t)encoder_e1_count;
    previous_right=(uint32_t)encoder_e2_count;
    previous_ms=uptime_ms;
    speed_left_cps=0.0f; speed_right_cps=0.0f; speed_filtered_cps=0.0f;
    speed_integral_deg=0.0f; speed_target_body_deg=0.0f; speed_state=0u;
    speed_error_cps=0.0f;
    speed_wheel_difference_cps=0.0f; speed_wheel_correction_percent=0.0f;
    speed_wheel_integral_percent=0.0f;
#ifndef PARALLEL_PID_TRIAL
    sync_previous_error_cm=0.0f;sync_tracking=0u;
#endif
    speed_wheel_motion_blend=0.0f;
#ifdef PARALLEL_PID_TRIAL
    parallel_reset();
    position_hold_reset();
#endif
    __set_PRIMASK(saved);
}
void speed_loop_init(void)
{
    speed_kp=SPEED_DEFAULT_KP; speed_ki=SPEED_DEFAULT_KI;
    speed_tilt_limit_deg=SPEED_DEFAULT_TILT_DEG; speed_update_count=0u;
#ifdef PARALLEL_PID_TRIAL
    speed_parallel_kp=PAR_TRIAL_SPEED_KP;speed_parallel_ki=PAR_TRIAL_SPEED_KI;
    speed_parallel_limit_percent=PAR_TRIAL_SPEED_LIMIT;
#endif
    speed_loop_reset();
}
static void poll_locked(void)
{
    uint32_t left,right,now,dt_ms;
    float dt,mean;
#ifndef PARALLEL_PID_TRIAL
    float request,sync_kp,sync_limit;
#endif
#ifndef PARALLEL_PID_TRIAL
    float p,increment,candidate,delta,limit;
#endif

#ifndef PARALLEL_PID_TRIAL
    float sync_increment,sync_candidate,sync_p;
    uint32_t unwind;
#endif
    if(pd_window_active==0u) return;
    task_motion_poll(uptime_ms);
    left=(uint32_t)encoder_e1_count; right=(uint32_t)encoder_e2_count; now=uptime_ms;
    dt_ms=(uint32_t)(now-previous_ms);
    if(dt_ms<SPEED_PERIOD_MS) return;
    if(dt_ms>100u ||
#ifdef PARALLEL_PID_TRIAL
       !speed_loop_parallel_config_valid(speed_parallel_kp,speed_parallel_ki,speed_parallel_limit_percent)) {
#else
       !speed_loop_config_valid(speed_kp,speed_ki,speed_tilt_limit_deg)) {
#endif
        speed_loop_reset(); speed_state=2u; return;
    }
    dt=(float)dt_ms*0.001f;
    /* Physical forward is E1 negative, E2 positive. Modulo deltas handle wrap. */
    speed_left_cps=-(float)encoder_signed32(left-previous_left)/dt;
    speed_right_cps=(float)encoder_signed32(right-previous_right)/dt;
    previous_left=left; previous_right=right; previous_ms=now;
    mean=0.5f*(speed_left_cps+speed_right_cps);
    speed_filtered_cps += dt/(SPEED_FILTER_SECONDS+dt)*(mean-speed_filtered_cps);
    speed_wheel_difference_cps += dt/(WHEEL_SYNC_FILTER_SECONDS+dt)*
        (speed_right_cps-speed_left_cps-speed_wheel_difference_cps);
#ifdef PARALLEL_PID_TRIAL
    parallel_wheel_sync(dt);
    position_hold_poll(left,right,now);
#else
    /* Right faster in signed physical coordinates -> more forward torque
       on left, less on right. Board conversion handles reverse symmetrically. */
    /* Strong tracking belongs to commanded travel, not the settling wait.
       Blend continuously to the proven standing response as the reference
       goes to zero. The original correction slew also covers state changes. */
    speed_wheel_motion_blend=0.0f;
    if(task_state==TASK_FORWARD || task_state==TASK_DISTANCE || task_state==TASK_BRAKING){
        request=task_reference_cps<0.0f ? -task_reference_cps : task_reference_cps;
        speed_wheel_motion_blend=clip(request/(TASK_COUNTS_PER_CM*TASK_WHEEL_SYNC_FULL_SPEED_CMS),1.0f);
    }
    sync_kp=WHEEL_SYNC_KP+(TASK_WHEEL_SYNC_KP-WHEEL_SYNC_KP)*speed_wheel_motion_blend;
    sync_limit=WHEEL_SYNC_LIMIT_PERCENT+(TASK_WHEEL_SYNC_LIMIT_PERCENT-WHEEL_SYNC_LIMIT_PERCENT)*speed_wheel_motion_blend;
    if(speed_wheel_motion_blend>0.0f){
        if(!sync_tracking || sync_run!=task_run_count){
            sync_tracking=1u;sync_run=task_run_count;
            sync_previous_error_cm=task_wheel_error_cm;speed_wheel_integral_percent=0.0f;
        }
        sync_increment=TASK_WHEEL_SYNC_POSITION_GAIN*(task_wheel_error_cm-sync_previous_error_cm);
        sync_previous_error_cm=task_wheel_error_cm;
        sync_candidate=clip(speed_wheel_integral_percent+sync_increment,TASK_WHEEL_SYNC_INTEGRAL_LIMIT);
        sync_p=sync_kp*speed_wheel_difference_cps*0.001f;
        unwind=(speed_wheel_integral_percent>0.0f && sync_increment<0.0f) ||
               (speed_wheel_integral_percent<0.0f && sync_increment>0.0f);
        /* Do not accumulate a steering debt while balance has no usable PWM.
           Saturation may always unwind. The board reports actual differential
           clipping in signed forward coordinates, including reverse drive. */
        if(unwind || (observed_stby && !balance_output_limited && !hold_actuator_limited &&
           !((hold_sync_saturation_error_counts>0 && sync_increment>0.0f) ||
             (hold_sync_saturation_error_counts<0 && sync_increment<0.0f)) &&
           !((sync_p+sync_candidate*speed_wheel_motion_blend>sync_limit && sync_increment>0.0f) ||
             (sync_p+sync_candidate*speed_wheel_motion_blend < -sync_limit && sync_increment<0.0f))))
            speed_wheel_integral_percent=sync_candidate;
    }else{
        sync_tracking=0u;speed_wheel_integral_percent=0.0f;
    }
    request=clip(sync_kp*speed_wheel_difference_cps*0.001f+
                 speed_wheel_integral_percent*speed_wheel_motion_blend,sync_limit);
    speed_wheel_correction_percent += clip(request-speed_wheel_correction_percent,
                                          WHEEL_SYNC_SLEW_PPS*dt);
#endif
#ifdef PARALLEL_PID_TRIAL
    if(!speed_loop_command_enabled()) {parallel_reset();speed_state=0u;}
    else {
        speed_effective_reference_cps=task_motion_control_reference()+position_hold_target_cms*TASK_COUNTS_PER_CM;
        speed_error_cps=speed_filtered_cps-speed_effective_reference_cps;
        parallel_dt=dt; /* Integrate once, with the current angle PD term. */
        speed_state=1u;
    }
    ++speed_update_count;
#else
    limit=speed_tilt_limit_deg;
    if(limit==0.0f || (speed_kp==0.0f && speed_ki==0.0f)) {
        speed_integral_deg=0.0f; speed_target_body_deg=0.0f; speed_state=0u;
        ++speed_update_count; return;
    }
    speed_error_cps=speed_filtered_cps-task_motion_control_reference();
    p=speed_kp*speed_error_cps*0.001f;
    increment=speed_ki*speed_error_cps*0.001f*dt;
    candidate=clip(speed_integral_deg+increment,limit);
    /* Do not wind up against target limits or the inner actuator limit. */
    if(!balance_output_limited && !hold_actuator_limited && imu_body_pitch_deg>=-8.0f && imu_body_pitch_deg<=8.0f &&
       !((p+candidate>limit && increment>0.0f) ||
         (p+candidate < -limit && increment<0.0f))) speed_integral_deg=candidate;
    request=clip(p+speed_integral_deg,limit);
    delta=clip(request-speed_target_body_deg,SPEED_TARGET_SLEW_DPS*dt);
    /* Forward velocity -> positive body target (back lean). Angle inner loop
       first moves wheels under the mass; this is not a direct brake PWM. */
    speed_target_body_deg=clip(speed_target_body_deg+delta,limit);
    ++speed_update_count; speed_state=1u;
#endif
}
void speed_loop_poll(void)
{
    uint32_t saved=__get_PRIMASK();
    __disable_irq();
    poll_locked();
    __set_PRIMASK(saved);
}
#endif

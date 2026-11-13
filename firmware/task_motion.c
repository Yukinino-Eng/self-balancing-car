#include "task_motion.h"
#ifdef PD_HOLD_TEST
#include "speed_loop.h"
#include "pd_window.h"
#include "imu.h"
#include "encoder.h"
#include "board.h"
const volatile uint32_t task_continuous=1u,task_duration_ms=TASK_DURATION_MS;
const volatile uint32_t task_direction_reversed=1u;
const volatile uint32_t task_stop_ramp_ms=TASK_STOP_RAMP_MS;
/* Task forward is the former rear. The balance/encoder frame is unchanged. */
float task_motion_control_reference(void){return -task_reference_cps;}
volatile float task_speed_cms=TASK_DEFAULT_SPEED_CMS,task_reference_cps,task_distance_cm;
volatile float task_left_distance_cm,task_right_distance_cm,task_wheel_error_cm;
volatile float task_requested_distance_cm=TASK_DEFAULT_REVERSE_CM;
volatile float task_target_distance_cm,task_distance_error_cm;
volatile uint32_t task_mode,task_finish_reason,task_distance_timeout_ms;
volatile uint32_t task_distance_holding;
volatile uint32_t task_ramp_ms=TASK_DEFAULT_RAMP_MS,task_state,task_command,task_elapsed_ms,task_remaining_ms;
volatile uint32_t task_request_id,task_result_id,task_result,task_run_count,task_completed_count;
const volatile uint32_t task_mailbox_api=1u;
volatile uint32_t task_panel_command,task_panel_request_id,task_panel_expected_epoch;
volatile uint32_t task_panel_result_id,task_panel_result,task_cancel_epoch;
static uint32_t start_ms,ramp_ms,brake_ms,brake_ramp_ms,settle_ms,settle_tracking,profile_ms;
static void panel_ack(uint32_t id,uint32_t result)
{task_panel_result=result;__DMB();task_panel_result_id=id;}
static wheel_origin_t origin;
static float cruise_cps,brake_cps;
#ifdef TASK_HAS_OBSTACLE
#include "peripherals.h"
volatile uint32_t avoid_state,avoid_reason,avoid_hold_remaining_ms,avoid_stop_range_mm;
volatile uint32_t avoid_run_count,avoid_completed_count;
volatile float avoid_reverse_cm;
static uint32_t avoid_start_ms,avoid_phase_ms,avoid_hold_ms,avoid_hold_tracking;
static uint32_t avoid_good_ms,avoid_good_mm,avoid_have_range;
uint32_t task_motion_avoid_active(void)
{return (avoid_state>=AVOID_APPROACH && avoid_state<=AVOID_REVERSE) || avoid_state==AVOID_ABORTING;}
static void avoid_clear(void)
{avoid_state=AVOID_OFF;avoid_reason=AVOID_REASON_NONE;avoid_hold_remaining_ms=0u;avoid_have_range=0u;avoid_hold_tracking=0u;}
static uint32_t avoid_range(uint32_t now)
{
    if(tof_valid && tof_range_mm && (uint32_t)(now-tof_last_ms)<=AVOID_RANGE_STALE_MS){
        avoid_good_ms=tof_last_ms;avoid_good_mm=tof_range_mm;avoid_have_range=1u;
    }
    return avoid_have_range && (uint32_t)(now-avoid_good_ms)<=AVOID_RANGE_STALE_MS;
}
#endif
#ifdef LEARN_SIMULATOR
uint32_t task_sim_timed;
uint32_t task_sim_manual_distance;
#endif
uint32_t task_motion_config_valid(float speed,uint32_t ramp)
{
    return speed>=0.5f && speed<=10.0f && ramp>=300u && ramp<=2000u;
}
uint32_t task_motion_distance_valid(float distance)
{
    return (distance>=5.0f && distance<=20.0f) || (distance>=-20.0f && distance<=-5.0f);
}
uint32_t task_motion_is_continuous(void)
{
#ifdef LEARN_SIMULATOR
    return task_sim_timed==0u;
#else
    return 1u;
#endif
}
static float smooth(uint32_t t,uint32_t duration)
{
    float q;
    if(t>=duration)return 1.0f;
    q=(float)t/(float)duration;return q*q*(3.0f-2.0f*q);
}
static float clip(float value,float limit)
{
    return value>limit ? limit : value < -limit ? -limit : value;
}
static uint32_t stopped(void)
{
    return speed_filtered_cps>=-TASK_COUNTS_PER_CM && speed_filtered_cps<=TASK_COUNTS_PER_CM &&
           imu_body_pitch_deg>=-2.0f && imu_body_pitch_deg<=2.0f &&
           imu_pitch_rate_dps>=-20.0f && imu_pitch_rate_dps<=20.0f;
}
static void complete(uint32_t reason)
{
    task_state=TASK_DONE;task_finish_reason=reason;task_reference_cps=0.0f;task_remaining_ms=0u;
    if(reason==TASK_FINISH_TIME || reason==TASK_FINISH_DISTANCE)++task_completed_count;
    settle_tracking=0u;
}
static void brake(uint32_t now,uint32_t reason)
{
    brake_ms=now;brake_cps=task_reference_cps;task_state=TASK_BRAKING;
    brake_ramp_ms=ramp_ms<TASK_STOP_RAMP_MS?ramp_ms:TASK_STOP_RAMP_MS;
    task_remaining_ms=brake_ramp_ms;task_finish_reason=reason;settle_tracking=0u;
#ifdef PARALLEL_PID_TRIAL
    speed_loop_position_hold_at(0.0f);
#endif
}
void task_motion_end_session(void)
{
    ++task_cancel_epoch;task_panel_command=0u;
    task_command=0u;task_state=TASK_IDLE;task_reference_cps=0.0f;task_remaining_ms=0u;
    settle_tracking=0u;task_distance_holding=0u;
#ifdef TASK_HAS_OBSTACLE
    avoid_clear();
#endif
}
void task_motion_begin_session(void)
{
    task_motion_end_session();task_state=TASK_BALANCE;
    task_elapsed_ms=0u;task_distance_cm=0.0f;
    task_left_distance_cm=0.0f;task_right_distance_cm=0.0f;task_wheel_error_cm=0.0f;
    task_mode=TASK_MODE_TIMED;task_finish_reason=TASK_FINISH_NONE;
    task_target_distance_cm=0.0f;task_distance_error_cm=0.0f;
}
void task_motion_init(void)
{
    task_run_count=0u;task_completed_count=0u;task_request_id=0u;task_result_id=0u;task_result=0u;
    task_elapsed_ms=0u;task_distance_cm=0.0f;task_motion_end_session();
    task_cancel_epoch=0u;task_panel_request_id=0u;task_panel_expected_epoch=0u;
    task_panel_result_id=0u;task_panel_result=TASK_RESULT_IDLE;
    task_left_distance_cm=0.0f;task_right_distance_cm=0.0f;task_wheel_error_cm=0.0f;
    task_requested_distance_cm=TASK_DEFAULT_REVERSE_CM;
    task_mode=TASK_MODE_TIMED;task_finish_reason=TASK_FINISH_NONE;task_distance_timeout_ms=0u;
    task_target_distance_cm=0.0f;task_distance_error_cm=0.0f;
#ifdef TASK_HAS_OBSTACLE
    avoid_run_count=0u;avoid_completed_count=0u;avoid_reverse_cm=TASK_DEFAULT_REVERSE_CM;avoid_stop_range_mm=0u;
#endif
#ifdef LEARN_SIMULATOR
    task_sim_timed=0u;task_sim_manual_distance=0u;
#endif
}
/* Caller is the button ISR or the existing control critical section. */
void task_motion_request(uint32_t command)
{
    if(task_command!=0u && command!=9u)return;
    if(command==9u && task_command!=9u)++task_cancel_epoch;
    task_request_id=task_result_id+1u;task_command=command;
}
static void update_profile(uint32_t now)
{
    uint32_t elapsed,dt_ms;
    float requested,step,error_abs;
    wheel_distance_t distance;
    /* Measure before computing a distance command; no stale position frame. */
    if(task_state>=TASK_FORWARD){
        wheel_odometry_measure(&origin,(uint32_t)encoder_e1_count,(uint32_t)encoder_e2_count,&distance);
        task_left_distance_cm=-distance.left_cm;task_right_distance_cm=-distance.right_cm;
        task_distance_cm=-distance.mean_cm;task_wheel_error_cm=-distance.difference_cm;
        task_distance_error_cm=task_mode==TASK_MODE_DISTANCE ? task_target_distance_cm-task_distance_cm : 0.0f;
    }
#ifdef TASK_HAS_OBSTACLE
    if(task_state==TASK_APPROACH){
        elapsed=(uint32_t)(now-start_ms);task_elapsed_ms=elapsed;
        task_remaining_ms=elapsed<AVOID_APPROACH_TIMEOUT_MS?AVOID_APPROACH_TIMEOUT_MS-elapsed:0u;
        task_reference_cps=cruise_cps*smooth(elapsed,ramp_ms);
    }else
#endif
    if(task_state==TASK_FORWARD){
        elapsed=(uint32_t)(now-start_ms);
        task_elapsed_ms=elapsed<TASK_DURATION_MS?elapsed:TASK_DURATION_MS;
        task_remaining_ms=TASK_DURATION_MS-task_elapsed_ms;
        if(elapsed>=TASK_DURATION_MS){
            task_reference_cps=0.0f;task_state=TASK_SETTLING;settle_tracking=0u;task_finish_reason=TASK_FINISH_TIME;
#ifdef PARALLEL_PID_TRIAL
            speed_loop_position_hold_at(0.0f);
#endif
        }
        else if(elapsed<ramp_ms)task_reference_cps=cruise_cps*smooth(elapsed,ramp_ms);
        else if(elapsed>TASK_DURATION_MS-ramp_ms)
            task_reference_cps=cruise_cps*(1.0f-smooth(elapsed-(TASK_DURATION_MS-ramp_ms),ramp_ms));
        else task_reference_cps=cruise_cps;
    }else if(task_state==TASK_DISTANCE){
        elapsed=(uint32_t)(now-start_ms);task_elapsed_ms=elapsed;
        task_remaining_ms=elapsed<task_distance_timeout_ms ? task_distance_timeout_ms-elapsed : 0u;
        if(elapsed>=task_distance_timeout_ms)brake(now,TASK_FINISH_TIMEOUT);
        else{
            dt_ms=(uint32_t)(now-profile_ms);profile_ms=now;
            if(dt_ms>100u)dt_ms=100u; /* No catch-up command jump after a delayed poll. */
            error_abs=task_distance_error_cm<0.0f ? -task_distance_error_cm : task_distance_error_cm;
            /* Enter the stop band at 0.5 cm; do not chase encoder/pose motion
               around that boundary. Resume correction only beyond 0.8 cm. */
            if(task_distance_holding){
                if(error_abs>TASK_DISTANCE_RELEASE_CM)task_distance_holding=0u;
            }else if(error_abs<=TASK_DISTANCE_TOLERANCE_CM){
                task_distance_holding=1u;
#ifdef PARALLEL_PID_TRIAL
                /* New task error equals chassis current-minus-target. */
                speed_loop_position_hold_at(task_distance_error_cm);
#endif
            }
            requested=task_distance_holding ? 0.0f :
                      clip(TASK_DISTANCE_KP*task_distance_error_cm*TASK_COUNTS_PER_CM,cruise_cps);
            step=cruise_cps*(float)dt_ms/(float)ramp_ms;
            if(task_distance_holding)task_reference_cps=0.0f;
            else task_reference_cps+=clip(requested-task_reference_cps,step);
            /* Position tolerance alone never means the vehicle is stopped. */
            if(task_distance_holding && task_reference_cps==0.0f && stopped()){
                if(!settle_tracking){settle_ms=now;settle_tracking=1u;}
                else if((uint32_t)(now-settle_ms)>=500u)complete(TASK_FINISH_DISTANCE);
            }else settle_tracking=0u;
        }
    }else if(task_state==TASK_BRAKING){
        elapsed=(uint32_t)(now-brake_ms);
        task_reference_cps=brake_cps*(1.0f-smooth(elapsed,brake_ramp_ms));
        task_remaining_ms=elapsed<brake_ramp_ms?brake_ramp_ms-elapsed:0u;
        if(elapsed>=brake_ramp_ms){task_state=TASK_SETTLING;settle_tracking=0u;}
    }
    if(task_state==TASK_SETTLING){
        /* A zero reference is not proof of a stopped vehicle. */
        if(stopped()){
            if(!settle_tracking){settle_ms=now;settle_tracking=1u;}
            else if((uint32_t)(now-settle_ms)>=500u)complete(task_finish_reason);
        }else settle_tracking=0u;
    }
}
#ifdef TASK_HAS_OBSTACLE
static void avoid_finish(uint32_t unsettled)
{
    avoid_state=avoid_reason==AVOID_REASON_USER?AVOID_CANCELLED:AVOID_FAILED;
    avoid_hold_remaining_ms=0u;
    if(unsettled){
        /* End the mission without reporting a physically stopped vehicle. */
        task_state=TASK_FAILED;task_finish_reason=TASK_FINISH_SETTLE;
        task_reference_cps=0.0f;task_remaining_ms=0u;task_distance_holding=0u;settle_tracking=0u;
    }
}
static void avoid_abort(uint32_t now,uint32_t reason)
{
    if(avoid_state==AVOID_ABORTING){
        if(reason==AVOID_REASON_USER)avoid_reason=reason;
        return; /* Repeated cancel never extends the original deadline. */
    }
    avoid_reason=reason;avoid_hold_remaining_ms=0u;avoid_phase_ms=now;avoid_state=AVOID_ABORTING;
    if(reason==AVOID_REASON_SETTLE){avoid_finish(1u);return;}
    if(task_state==TASK_APPROACH || task_state==TASK_FORWARD || task_state==TASK_DISTANCE)
        brake(now,reason==AVOID_REASON_USER?TASK_FINISH_EARLY_STOP:reason==AVOID_REASON_SENSOR?TASK_FINISH_SENSOR:TASK_FINISH_BOUND);
    else if(task_state==TASK_DONE || task_state==TASK_BALANCE)
        avoid_state=reason==AVOID_REASON_USER?AVOID_CANCELLED:AVOID_FAILED;
}
static void avoid_reverse(uint32_t now)
{
    wheel_odometry_origin(&origin,(uint32_t)encoder_e1_count,(uint32_t)encoder_e2_count);
    start_ms=now;profile_ms=now;task_elapsed_ms=0u;task_reference_cps=0.0f;
    task_distance_cm=0.0f;task_left_distance_cm=0.0f;task_right_distance_cm=0.0f;task_wheel_error_cm=0.0f;
    task_mode=TASK_MODE_DISTANCE;task_state=TASK_DISTANCE;task_target_distance_cm=avoid_reverse_cm;
    task_distance_error_cm=avoid_reverse_cm;task_distance_holding=0u;task_finish_reason=TASK_FINISH_NONE;
    task_distance_timeout_ms=(uint32_t)(-avoid_reverse_cm*TASK_COUNTS_PER_CM*1000.0f/cruise_cps)+2u*ramp_ms+10000u;
    task_remaining_ms=task_distance_timeout_ms;settle_tracking=0u;avoid_state=AVOID_REVERSE;
}
static void avoid_poll(uint32_t now)
{
    uint32_t elapsed;
    if(!task_motion_avoid_active())return;
    if(avoid_state==AVOID_ABORTING){
        if(task_state==TASK_DONE)avoid_finish(0u);
        else if((uint32_t)(now-avoid_phase_ms)>=AVOID_SETTLE_TIMEOUT_MS)avoid_finish(1u);
        return;
    }
    if(!avoid_range(now)){avoid_abort(now,AVOID_REASON_SENSOR);return;}
    if(avoid_state==AVOID_APPROACH){
        if(avoid_good_mm<=AVOID_TRIGGER_MM){
            avoid_stop_range_mm=avoid_good_mm;avoid_phase_ms=now;avoid_state=AVOID_BRAKING;brake(now,TASK_FINISH_OBSTACLE);
        }else if((uint32_t)(now-avoid_start_ms)>=AVOID_APPROACH_TIMEOUT_MS)avoid_abort(now,AVOID_REASON_TIME);
        else if(task_distance_cm>=AVOID_MAX_FORWARD_CM)avoid_abort(now,AVOID_REASON_DISTANCE);
    }else if(avoid_state==AVOID_BRAKING){
        if(task_state==TASK_DONE){avoid_state=AVOID_HOLD;avoid_hold_ms=now;avoid_hold_tracking=1u;avoid_hold_remaining_ms=AVOID_HOLD_MS;}
        else if((uint32_t)(now-avoid_phase_ms)>=AVOID_SETTLE_TIMEOUT_MS)avoid_abort(now,AVOID_REASON_SETTLE);
    }else if(avoid_state==AVOID_HOLD){
        /* Allow normal small balance corrections. A material disturbance
         * restarts the five-second dwell after the vehicle is stopped again. */
        if(imu_body_pitch_deg<=-5.0f || imu_body_pitch_deg>=5.0f ||
           imu_pitch_rate_dps<=-35.0f || imu_pitch_rate_dps>=35.0f ||
           speed_filtered_cps<=-8.0f*TASK_COUNTS_PER_CM || speed_filtered_cps>=8.0f*TASK_COUNTS_PER_CM)
            avoid_hold_tracking=0u;
        if(!avoid_hold_tracking){
            avoid_hold_remaining_ms=AVOID_HOLD_MS;
            if(stopped()){avoid_hold_ms=now;avoid_hold_tracking=1u;}
        }else{
            elapsed=(uint32_t)(now-avoid_hold_ms);
            avoid_hold_remaining_ms=elapsed<AVOID_HOLD_MS?AVOID_HOLD_MS-elapsed:0u;
            if(elapsed>=AVOID_HOLD_MS)avoid_reverse(now);
        }
    }else if(avoid_state==AVOID_REVERSE){
        if(task_finish_reason==TASK_FINISH_TIMEOUT)avoid_abort(now,AVOID_REASON_REVERSE_TIMEOUT);
        else if(task_state==TASK_DONE){avoid_state=AVOID_COMPLETE;++avoid_completed_count;}
    }
}
#endif
void task_motion_poll(uint32_t now)
{
    uint32_t command=task_command,request,panel=task_panel_command;
    uint32_t panel_id=task_panel_request_id,from_panel=0u;
    if(!pd_window_active){
        if(panel)panel_ack(panel_id,TASK_INVALID);
        task_motion_end_session();return;
    }
    /* Host staging has its own mailbox. Button requests never overwrite its
     * fields; the command word is the host's last write/commit. Caller holds
     * the existing control IRQ lock. A cancel epoch revokes stale staging. */
    if(panel){
        task_panel_command=0u;
        if(panel==9u){
            if(command!=9u)++task_cancel_epoch;
            command=9u;from_panel=1u;
        }else if(panel==2u)panel_ack(panel_id,TASK_INVALID);
        else if(task_panel_expected_epoch!=task_cancel_epoch)panel_ack(panel_id,TASK_STALE);
        else if(command)panel_ack(panel_id,TASK_BUSY);
        else{command=panel;from_panel=1u;}
        if(from_panel){task_request_id=task_result_id+1u;task_command=command;}
    }
    update_profile(now);
    /* Cancel commands take priority over an automatic hold->reverse step. */
#ifdef TASK_HAS_OBSTACLE
    if(command!=9u)avoid_poll(now);
#endif
    if(command==0u)return;
    if(command==2u
#ifdef LEARN_SIMULATOR
       && !task_sim_manual_distance
#endif
    ){task_command=0u;task_result=TASK_INVALID;task_result_id=task_request_id;return;}
    task_command=0u;request=task_request_id;
#ifdef TASK_HAS_OBSTACLE
    if(command==9u && task_state==TASK_FAILED){
        task_motion_begin_session();task_result=TASK_ACCEPTED;
    }else if(command==9u && task_motion_avoid_active()){
        avoid_abort(now,AVOID_REASON_USER);task_result=TASK_ACCEPTED;
    }else if(task_motion_avoid_active())task_result=TASK_BUSY;
    else if(command==3u){
        avoid_have_range=0u;
        if(task_state!=TASK_BALANCE && task_state!=TASK_DONE)task_result=TASK_BUSY;
        else if(!speed_loop_command_enabled() || !task_motion_config_valid(task_speed_cms,task_ramp_ms) ||
                task_speed_cms>AVOID_MAX_SPEED_CMS || task_ramp_ms>AVOID_MAX_RAMP_MS ||
                !(task_requested_distance_cm>=-20.0f && task_requested_distance_cm<=-5.0f)){
            avoid_clear();avoid_state=AVOID_FAILED;avoid_reason=AVOID_REASON_CONFIG;task_result=TASK_INVALID;
        }else if(!avoid_range(now)){
            avoid_clear();avoid_state=AVOID_FAILED;avoid_reason=AVOID_REASON_SENSOR;task_result=TASK_INVALID;
        }
        else{
            avoid_clear();(void)avoid_range(now);avoid_reverse_cm=task_requested_distance_cm;
            avoid_state=AVOID_APPROACH;avoid_start_ms=now;avoid_stop_range_mm=0u;++avoid_run_count;
            ramp_ms=task_ramp_ms;cruise_cps=task_speed_cms*TASK_COUNTS_PER_CM;
            wheel_odometry_origin(&origin,(uint32_t)encoder_e1_count,(uint32_t)encoder_e2_count);
            start_ms=now;profile_ms=now;settle_tracking=0u;task_reference_cps=0.0f;
            task_distance_cm=0.0f;task_left_distance_cm=0.0f;task_right_distance_cm=0.0f;task_wheel_error_cm=0.0f;
            task_mode=TASK_MODE_TIMED;task_state=TASK_APPROACH;task_finish_reason=TASK_FINISH_NONE;
            task_elapsed_ms=0u;task_remaining_ms=AVOID_APPROACH_TIMEOUT_MS;++task_run_count;task_result=TASK_ACCEPTED;
            avoid_poll(now); /* Already near a wall: never command forward first. */
        }
    }else
#endif
    if(command==1u || command==2u){
        if(task_state!=TASK_BALANCE && task_state!=TASK_DONE)task_result=TASK_BUSY;
        else if(!task_motion_config_valid(task_speed_cms,task_ramp_ms) ||
                !speed_loop_command_enabled() ||
                (command==2u && !task_motion_distance_valid(task_requested_distance_cm)))task_result=TASK_INVALID;
        else{
#ifdef TASK_HAS_OBSTACLE
            avoid_clear();
#endif
            ramp_ms=task_ramp_ms;cruise_cps=task_speed_cms*TASK_COUNTS_PER_CM;
            start_ms=now;profile_ms=now;task_elapsed_ms=0u;task_remaining_ms=TASK_DURATION_MS;
            wheel_odometry_origin(&origin,(uint32_t)encoder_e1_count,(uint32_t)encoder_e2_count);
            task_left_distance_cm=0.0f;task_right_distance_cm=0.0f;task_wheel_error_cm=0.0f;
            task_distance_cm=0.0f;task_reference_cps=0.0f;task_state=TASK_FORWARD;
            task_mode=command==2u ? TASK_MODE_DISTANCE : TASK_MODE_TIMED;task_finish_reason=TASK_FINISH_NONE;
            task_distance_holding=0u;
            task_target_distance_cm=command==2u ? task_requested_distance_cm : 0.0f;
            task_distance_error_cm=task_target_distance_cm;
            if(command==2u){
                task_state=TASK_DISTANCE;
                task_distance_timeout_ms=(uint32_t)((task_target_distance_cm<0.0f ? -task_target_distance_cm : task_target_distance_cm)*1000.0f/task_speed_cms)+2u*ramp_ms+10000u;
                task_remaining_ms=task_distance_timeout_ms;
            }
            settle_tracking=0u;++task_run_count;task_result=TASK_ACCEPTED;
        }
    }else if(command==9u){
        if(task_state==TASK_FORWARD || task_state==TASK_DISTANCE){brake(now,TASK_FINISH_EARLY_STOP);task_result=TASK_ACCEPTED;}
        else if(task_state==TASK_BALANCE || task_state==TASK_DONE || task_state==TASK_BRAKING || task_state==TASK_SETTLING)task_result=TASK_ACCEPTED;
        else task_result=TASK_BUSY;
    }else task_result=TASK_INVALID;
    task_result_id=request;
    if(from_panel)panel_ack(panel_id,task_result);
}
#endif

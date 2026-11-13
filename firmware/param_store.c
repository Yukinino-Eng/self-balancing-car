#include "param_store.h"
#ifdef PD_HOLD_TEST
#include "board.h"
#include "runtime.h"
#include "pd_window.h"
#include "hold_tuning.h"
#include "local_control.h"
#include "imu.h"
#include "hold_config.h"
#include "task_motion.h"
#include <stddef.h>
#ifdef PARALLEL_PID_TRIAL
#include "speed_loop.h"
#endif
#define RECORD_MAGIC 0x50333150u
#define RECORD_VERSION 3u
#define RECORD_COMMIT 0xC0DEC0DEu
typedef struct {
    uint32_t magic,version,sequence,data[PARAM_WORDS],crc,commit;
#ifdef PARALLEL_PID_TRIAL
    /* Preserve the v3 prefix for cascade firmware. Its commit is still written last. */
    uint32_t parallel_magic;
    float parallel_kp,parallel_ki,parallel_limit;
    uint32_t parallel_crc;
#endif
} record_t;
#define RECORD_CRC_BYTES ((uint32_t)offsetof(record_t,crc))
#define RECORD_COMMIT_HALFWORD ((uint32_t)offsetof(record_t,commit)/2u)
#ifdef PARALLEL_PID_TRIAL
#define PARALLEL_RECORD_MAGIC 0x50495031u
static uint32_t valid_parallel_values(const record_t *r)
{
    return speed_loop_parallel_config_valid(r->parallel_kp,r->parallel_ki,r->parallel_limit);
}
#endif
volatile uint32_t param_store_command,param_store_status,param_store_valid,param_store_sequence;
volatile uint32_t param_store_busy,param_store_request_id,param_store_result_id;
volatile uint32_t param_store_dirty;
static uint32_t current_page;
static uint32_t current_legacy;
#ifdef LEARN_SIMULATOR
uint32_t param_store_sim_pages[2][256];
int32_t param_store_sim_fail_after=-1;
void param_store_sim_clear(void)
{
    uint32_t i,j;
    for(i=0u;i<2u;i++)for(j=0u;j<256u;j++)param_store_sim_pages[i][j]=0xFFFFFFFFu;
    param_store_sim_fail_after=-1;
}
#endif
static uint32_t crc32(const uint8_t *p,uint32_t length)
{
    uint32_t crc=0xFFFFFFFFu,i;
    while(length--){crc^=*p++;for(i=0u;i<8u;i++)crc=(crc>>1)^((crc&1u)?0xEDB88320u:0u);}
    return ~crc;
}
static uint32_t read_record(uint32_t page,record_t *out)
{
    uint32_t i;uint32_t *dst=(uint32_t *)out;
#ifdef LEARN_SIMULATOR
    const uint32_t *src=param_store_sim_pages[page];
#else
    const volatile uint32_t *src=(const volatile uint32_t *)(page?PARAM_PAGE_B:PARAM_PAGE_A);
#endif
    for(i=0u;i<sizeof(record_t)/4u;i++)dst[i]=src[i];
    /* P31/P32 v1: 13 payload words, CRC at16, commit at17.
       Verify the original bytes before normalizing to the v2 layout in RAM.
       Never rewrite flash during boot or discard the previous valid page. */
    if(dst[0]==RECORD_MAGIC &&
       ((dst[1]==1u && dst[17]==RECORD_COMMIT && dst[16]==crc32((const uint8_t *)out,64u)) ||
        (dst[1]==2u && dst[18]==RECORD_COMMIT && dst[17]==crc32((const uint8_t *)out,68u)))) {
        union{float f;uint32_t u;} speed;
        if(dst[1]==1u)out->data[13]=HOLD_COMMON_BOOST_COUNTS;
        speed.f=TASK_DEFAULT_SPEED_CMS;
        out->data[14]=speed.u;out->data[15]=TASK_DEFAULT_RAMP_MS;
        out->version=RECORD_VERSION;
        out->crc=crc32((const uint8_t *)out,RECORD_CRC_BYTES);
#ifdef PARALLEL_PID_TRIAL
        out->parallel_magic=0u;
#endif
        out->commit=RECORD_COMMIT;return 1u;
    }
    return 0u;
}
static uint32_t valid_record(const record_t *r)
{
    return r->magic==RECORD_MAGIC && r->version==RECORD_VERSION && r->commit==RECORD_COMMIT &&
        r->crc==crc32((const uint8_t *)r,RECORD_CRC_BYTES) && hold_tuning_record_valid(r->data);
}
#ifdef PARALLEL_PID_TRIAL
static uint32_t parallel_record_crc(const record_t *r)
{
    return crc32((const uint8_t *)r,(uint32_t)offsetof(record_t,parallel_crc));
}
static uint32_t valid_parallel_record(const record_t *r)
{
    return r->parallel_magic==PARALLEL_RECORD_MAGIC && r->parallel_crc==parallel_record_crc(r) &&
           valid_parallel_values(r);
}
#endif
void param_store_mark_dirty(void)
{
    record_t r;uint32_t data[PARAM_WORDS],i;
    param_store_dirty=1u;
    if(!param_store_valid || current_legacy)return;
    read_record(current_page,&r);if(!valid_record(&r))return;
    hold_tuning_export_record(data);
    for(i=0u;i<PARAM_WORDS;i++)if(data[i]!=r.data[i])return;
#ifdef PARALLEL_PID_TRIAL
    if(!valid_parallel_record(&r) || r.parallel_kp!=speed_parallel_kp ||
       r.parallel_ki!=speed_parallel_ki || r.parallel_limit!=speed_parallel_limit_percent)return;
#endif
    param_store_dirty=0u;
}
#ifndef LEARN_SIMULATOR
static uint32_t flash_ready(void)
{
    uint32_t budget=1000000u;
    while((FLASH->SR&1u)!=0u && --budget) { }
    return budget!=0u && (FLASH->SR&0x14u)==0u;
}
#endif
/* Write the legacy commit after the extension, so both readers see one commit. */
static uint32_t program_halfword_index(uint32_t i)
{
    const uint32_t count=sizeof(record_t)/2u;
    if(i>=count-2u)return RECORD_COMMIT_HALFWORD+i-(count-2u);
    return i<RECORD_COMMIT_HALFWORD?i:i+2u;
}
static uint32_t write_record(uint32_t page,const record_t *r)
{
    uint32_t i,index;
    const uint16_t *src=(const uint16_t *)r;
#ifdef LEARN_SIMULATOR
    uint16_t *dst=(uint16_t *)param_store_sim_pages[page];
    for(i=0u;i<256u;i++)param_store_sim_pages[page][i]=0xFFFFFFFFu;
    for(i=0u;i<sizeof(record_t)/2u;i++){
        if(param_store_sim_fail_after==0)return 0u;
        if(param_store_sim_fail_after>0)--param_store_sim_fail_after;
        index=program_halfword_index(i);dst[index]&=src[index];
    }
    return 1u;
#else
    uint32_t address=page?PARAM_PAGE_B:PARAM_PAGE_A;
    volatile uint16_t *dst=(volatile uint16_t *)address;
    FLASH->KEYR=0x45670123u;FLASH->KEYR=0xCDEF89ABu;
    FLASH->SR=0x34u;
    FLASH->CR=2u;FLASH->AR=address;FLASH->CR|=0x40u;
    board_watchdog_feed();
    if(!flash_ready()){FLASH->CR=0x80u;return 0u;}
    FLASH->CR=1u;
    /* Commit is last: a torn half-word or CRC never replaces the old page. */
    for(i=0u;i<sizeof(record_t)/2u;i++){
        index=program_halfword_index(i);board_watchdog_feed();dst[index]=src[index];
        if(!flash_ready() || dst[index]!=src[index]){FLASH->CR=0x80u;return 0u;}
    }
    FLASH->CR=0x80u;return 1u;
#endif
}
void param_store_init(void)
{
    record_t a,b;uint32_t va,vb,la,lb;
    param_store_command=0u;param_store_status=STORE_IDLE;param_store_busy=0u;
    param_store_request_id=0u;param_store_result_id=0u;param_store_valid=0u;param_store_sequence=0u;current_page=1u;param_store_dirty=1u;
    current_legacy=0u;
    la=read_record(0u,&a);lb=read_record(1u,&b);va=valid_record(&a);vb=valid_record(&b);
    if(!va && !vb)return;
    current_page=vb && (!va || (b.sequence!=a.sequence && (uint32_t)(b.sequence-a.sequence)<0x80000000u))?1u:0u;
    current_legacy=current_page?lb:la;
    {
        const record_t *r=current_page?&b:&a;
        if(!hold_tuning_load_record(r->data))return;
        param_store_valid=1u;param_store_sequence=r->sequence;
#ifdef PARALLEL_PID_TRIAL
        if(valid_parallel_record(r)){
            speed_parallel_kp=r->parallel_kp;
            speed_parallel_ki=r->parallel_ki;
            speed_parallel_limit_percent=r->parallel_limit;
        }
#endif
        param_store_mark_dirty();
    }
}
void param_store_poll(void)
{
    uint32_t saved,command,request,page;record_t r,verify;
    if(!param_store_command)return;
    saved=__get_PRIMASK();__disable_irq();command=param_store_command;param_store_command=0u;
    request=param_store_request_id;
    if(pd_window_active || local_state==LOCAL_PREPARING || local_state==LOCAL_REQUESTED)param_store_status=STORE_BUSY;
    else if(command!=1u || !hold_tuning_active_valid()
#ifdef PARALLEL_PID_TRIAL
            || !speed_loop_parallel_config_valid(speed_parallel_kp,speed_parallel_ki,speed_parallel_limit_percent)
#endif
            )param_store_status=STORE_INVALID;
    else{
        param_store_mark_dirty();
        if(param_store_valid && !param_store_dirty){param_store_status=STORE_SAVED;param_store_result_id=request;__set_PRIMASK(saved);return;}
        board_motor_off();param_store_busy=1u;
        r.magic=RECORD_MAGIC;r.version=RECORD_VERSION;r.sequence=param_store_sequence+1u;
        hold_tuning_export_record(r.data);r.crc=crc32((const uint8_t *)&r,RECORD_CRC_BYTES);r.commit=RECORD_COMMIT;
#ifdef PARALLEL_PID_TRIAL
        r.parallel_magic=PARALLEL_RECORD_MAGIC;
        r.parallel_kp=speed_parallel_kp;r.parallel_ki=speed_parallel_ki;r.parallel_limit=speed_parallel_limit_percent;
        r.parallel_crc=parallel_record_crc(&r);
#endif
        page=current_page^1u;
        if(write_record(page,&r)){
            read_record(page,&verify);
            if(valid_record(&verify) && verify.sequence==r.sequence
#ifdef PARALLEL_PID_TRIAL
               && valid_parallel_record(&verify) && verify.parallel_crc==r.parallel_crc
#endif
               ){
                current_page=page;param_store_valid=1u;param_store_sequence=r.sequence;param_store_status=STORE_SAVED;
                current_legacy=0u;
                param_store_dirty=0u;
            }else param_store_status=STORE_IO_ERROR;
        }else param_store_status=STORE_IO_ERROR;
        param_store_busy=0u;imu_fusion_invalidate();board_watchdog_feed();
    }
    param_store_result_id=request;__set_PRIMASK(saved);
}
#endif

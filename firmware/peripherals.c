#if defined(__CC_ARM)
#pragma O2
#endif
#include "peripherals.h"
#if defined(CAR_OLED_RANGE) || defined(PERIPHERAL_SELFTEST)
#include "board.h"
#include "runtime.h"
#include "pd_window.h"
#include "imu.h"
#include "vl53l1x_boot_config.h"
#include <string.h>
volatile uint32_t tof_state,tof_sensor_id,tof_range_mm,tof_range_status,tof_valid;
volatile uint32_t tof_sample_count,tof_last_ms,tof_error,tof_error_count;
volatile uint32_t oled_state,oled_address,oled_on,oled_error_count;
volatile uint32_t peripheral_io_count,peripheral_io_max_us,obstacle_indicator_on;
volatile uint32_t tof_mux_readback,tof_start_readback,tof_stream_count,tof_ready_raw;
volatile uint32_t peripheral_timeout_count,peripheral_fail_address,peripheral_fail_reg;
volatile uint32_t tof_reset_count,tof_boot_readback;
volatile uint32_t peripheral_io_retry_count;
volatile uint32_t peripheral_bus_clear_count,peripheral_bus_clear_fail_count,peripheral_fail_kind;
volatile uint32_t tof_bus_clear_count,tof_bus_clear_fail_count,tof_transfer_error_count,tof_timeout_count,tof_fail_kind,tof_io_max_us;
volatile uint32_t oled_bus_clear_count,oled_bus_clear_fail_count,oled_transfer_error_count,oled_timeout_count,oled_fail_kind,oled_io_max_us;
volatile uint32_t oled_dirty_chunks,oled_render_count;
static uint8_t number_frame[384];
static uint8_t tof_io_failures,oled_io_failures,last_io_tof;
static uint32_t tof_due,tof_deadline,oled_due,last_poll,render_ms,rendered_mm;
static uint16_t config_offset,clock_pll;
static uint8_t tof_phase,raw_status,oled_phase,oled_init_step,oled_scan_byte;
static uint16_t pending_mm,oled_pending_chunk,oled_cursor;
static uint8_t oled_cursor_valid,oled_data_pending,oled_pixel_retry,frame[1024],dirty[32];
static const uint8_t oled_commands[][2]={
 {0xae,0},{0xd5,0x80},{0xa8,0x3f},{0xd3,0},{0x40,0},{0x8d,0x14},
 {0x20,2},{0xa1,0},{0xc8,0},{0xda,0x12},{0x81,0x7f},{0xd9,0xf1},
 {0xdb,0x40},{0xa4,0},{0xa6,0}
};
static const uint8_t oled_command_lengths[]={1,2,2,2,1,2,2,1,1,2,2,2,2,1,1};
/* The real clock wait is also exercised with an injected preemption in
 * the simulator; the sensor model alone cannot catch this edge-timing race. */
static uint32_t io_start,wire_failure;
#ifdef PERIPHERAL_SELFTEST
static uint32_t sim_wire_ticks,sim_clock_mode,sim_clock_reads;
static __forceinline uint32_t wire_now(void){return sim_wire_ticks;}
static __forceinline void wire_clock_release(void){}
#ifdef REVIEW_ONLY_SELFTEST
static __forceinline void sda(uint32_t high){(void)high;}
static __forceinline void scl_low(void){}
static __forceinline void wire_release_all(void){}
static __forceinline uint32_t wire_bus_high(void){return sim_clock_mode!=2u;}
#endif
static __forceinline uint32_t wire_clock_high(void)
{
    ++sim_clock_reads;
    if(sim_clock_mode==2u){sim_wire_ticks+=80u;return 0u;}
    if((sim_clock_mode==1u || sim_clock_mode==3u) && sim_clock_reads==1u){
        sim_wire_ticks=sim_clock_mode==1u?800u:PERIPH_TRANSFER_MAX_US*8u+8u;
        return 0u; /* IRQ runs after this low observation; pin rises in IRQ. */
    }
    return 1u;
}
static __forceinline void delay_bit(void){sim_wire_ticks+=24u;}
#else
/* AD19-08G VIN pull-ups follow its 5 V supply. Keep it off the OLED bus.
 * PA9/PA10 are FT pins; open drain with no internal pull-up/down. */
static uint32_t scl_pin=1u<<4,sda_pin=1u<<5;
#define SCL_PIN scl_pin
#define SDA_PIN sda_pin
/* At 8 MHz, per-bit helper calls used most of the transaction budget.
 * Inline the GPIO/timer primitives; retain the 3 us phase delay and deadline. */
/* This fixed 3 us wait cannot block on external hardware. Check deadlines at
 * clock edges, avoiding a second DWT read on every spin-loop iteration. */
static __forceinline void delay_bit(void){uint32_t t=DWT->CYCCNT;while((uint32_t)(DWT->CYCCNT-t)<24u){} }
static __forceinline void sda(uint32_t high){if(high)GPIOA->BSRR=SDA_PIN;else GPIOA->BRR=SDA_PIN;}
static __forceinline void scl_low(void){GPIOA->BRR=SCL_PIN;}
static __forceinline uint32_t wire_now(void){return DWT->CYCCNT;}
static __forceinline void wire_clock_release(void){GPIOA->BSRR=SCL_PIN;}
static __forceinline void wire_release_all(void){GPIOA->BSRR=SCL_PIN|SDA_PIN;}
static __forceinline uint32_t wire_bus_high(void)
{return (GPIOA->IDR&(SCL_PIN|SDA_PIN))==(SCL_PIN|SDA_PIN);}
static __forceinline uint32_t wire_clock_high(void){return (GPIOA->IDR&SCL_PIN)!=0u;}
#endif
static __forceinline uint32_t in_budget(void){return (uint32_t)(wire_now()-io_start)<PERIPH_TRANSFER_MAX_US*8u;}
static __forceinline uint32_t scl_high(void)
{
    uint32_t t=wire_now();wire_clock_release();
    while(!wire_clock_high()){
        if(!in_budget()){wire_failure=PERIPH_FAIL_DEADLINE;return 0u;}
        if((uint32_t)(wire_now()-t)>400u){
            /* A SysTick may run after the low read. Re-sample before calling
             * this clock stretching: the slave may have released it in IRQ. */
            if(wire_clock_high())break;
            wire_failure=PERIPH_FAIL_CLOCK_LOW;return 0u;
        }
    }
    delay_bit();
    if(!in_budget()){wire_failure=PERIPH_FAIL_DEADLINE;return 0u;}
    return 1u;
}

#if !defined(PERIPHERAL_SELFTEST) || defined(REVIEW_ONLY_SELFTEST)
/* A transaction succeeds only after its STOP and released lines succeed.
 * This exact finalization path is exercised by the simulator as well. */
static uint32_t stop_wire(void)
{
    uint32_t ok;
    scl_low();sda(0u);delay_bit();ok=scl_high();sda(1u);delay_bit();
    wire_release_all();
    if(!wire_bus_high()){
        if(!wire_failure)wire_failure=PERIPH_FAIL_DATA_LOW;
        return 0u;
    }
    return ok && in_budget();
}
static uint32_t finish_transfer(uint32_t address,uint32_t reg,uint32_t ok)
{
    uint32_t elapsed,kind;
    if(!stop_wire())ok=0u;
    elapsed=(uint32_t)(wire_now()-io_start)/8u;
    if(elapsed>peripheral_io_max_us)peripheral_io_max_us=elapsed;
    if(address==0x29u){if(elapsed>tof_io_max_us)tof_io_max_us=elapsed;}
    else if(elapsed>oled_io_max_us)oled_io_max_us=elapsed;
    if(!ok || elapsed>PERIPH_TRANSFER_MAX_US){
        kind=wire_failure?wire_failure:PERIPH_FAIL_DEADLINE;
        if(elapsed>PERIPH_TRANSFER_MAX_US || kind==PERIPH_FAIL_DEADLINE){
            ++peripheral_timeout_count;kind=PERIPH_FAIL_DEADLINE;
            if(address==0x29u)++tof_timeout_count;else ++oled_timeout_count;
        }
        peripheral_fail_address=address;peripheral_fail_reg=reg;peripheral_fail_kind=kind;
        if(address==0x29u){++tof_transfer_error_count;tof_fail_kind=kind;}
        else{++oled_transfer_error_count;oled_fail_kind=kind;}
        return 0u;
    }
    return 1u;
}

#endif
#ifdef PERIPHERAL_SELFTEST
static uint32_t sim_present=3u,sim_wrong_id,sim_never_ready,sim_bad_status;
static uint32_t sim_distance=150u,sim_ready_at,sim_config_writes,sim_alt_oled;
static uint32_t sim_bus_fail,sim_scheduler_overrun;
static uint32_t sim_boot_at,sim_preboot_config,sim_reset_writes;
static uint32_t sim_tof_nacks,sim_oled_nacks,sim_oled_data_nacks;
static uint32_t transfer(uint8_t address,const uint8_t *tx,uint32_t txlen,uint8_t *rx,uint32_t rxlen)
{
    uint16_t reg=0u;uint32_t value=0u;
    ++peripheral_io_count;if(peripheral_io_max_us<200u)peripheral_io_max_us=200u;
    if(sim_bus_fail)return 0u;
    if((address==0x3cu || address==0x3du) && sim_oled_nacks){--sim_oled_nacks;return 0u;}
    if((address==0x3cu || address==0x3du) && txlen && tx[0]==0x40u && sim_oled_data_nacks){--sim_oled_data_nacks;return 0u;}
    if(address==0x29u && sim_tof_nacks){--sim_tof_nacks;return 0u;}
    if(address==0x3cu || address==0x3du)
        return (sim_present&2u) && address==(sim_alt_oled?0x3du:0x3cu);
    if(address!=0x29u || !(sim_present&1u))return 0u;
    if(txlen>=2u)reg=(uint16_t)(((uint16_t)tx[0]<<8)|tx[1]);
    if(rxlen){
        if(reg==0x010fu)value=sim_wrong_id?0xeeaau:0xeaccu;
        else if(reg==0x00e5u)value=(int32_t)(uptime_ms-sim_boot_at)>=0;
        else if(reg==0x0031u)value=!sim_never_ready && (int32_t)(uptime_ms-sim_ready_at)>=0;
        else if(reg==0x00deu)value=256u;
        else if(reg==0x0089u)value=sim_bad_status?4u:9u;
        else if(reg==0x0096u)value=sim_distance;
        else if(reg==0x0030u)value=1u;
        else if(reg==0x0087u)value=0x40u;
        else if(reg==0x008bu)value=tof_sample_count&255u;
        if(rxlen==2u){rx[0]=(uint8_t)(value>>8);rx[1]=(uint8_t)value;}else rx[0]=(uint8_t)value;
    }else if(txlen>2u){
        ++sim_config_writes;
        if(reg==0u)++sim_reset_writes;
        if(reg==0u && tx[2]==1u)sim_boot_at=uptime_ms+8u;
        if(reg!=0u && (int32_t)(uptime_ms-sim_boot_at)<0)sim_preboot_config=1u;
        if(reg==0x0087u && tx[2]==0x40u)sim_ready_at=uptime_ms+TOF_BUDGET_MS;
        if(reg==0x0086u && tx[2]==1u)sim_ready_at=uptime_ms+TOF_PERIOD_MS;
    }
    return 1u;
}
#else
static uint32_t start_wire(void)
{
    sda(1u);delay_bit();if(!scl_high())return 0u;
    if(!(GPIOA->IDR&SDA_PIN)){wire_failure=PERIPH_FAIL_DATA_LOW;return 0u;}
    sda(0u);delay_bit();scl_low();return in_budget();
}
/* Recover only a data line stuck low between transactions. Keep SDA released,
 * supply at most nine clocks and a STOP (NXP UM10204 bus clear). All waits share
 * the transaction deadline; never reset the MCU/sensor or touch the other bus. */
static uint32_t idle_wire(void)
{
    uint32_t i,stopped;
    sda(1u);delay_bit();
    if(GPIOA->IDR&SDA_PIN)return 1u;
    ++peripheral_bus_clear_count;
    if(scl_pin==(1u<<9))++tof_bus_clear_count;else ++oled_bus_clear_count;
    for(i=0u;i<9u && !(GPIOA->IDR&SDA_PIN);++i){
        scl_low();delay_bit();if(!scl_high())break;
    }
    stopped=stop_wire();
    if(stopped && wire_bus_high() && in_budget())return 1u;
    ++peripheral_bus_clear_fail_count;
    if(scl_pin==(1u<<9))++tof_bus_clear_fail_count;else ++oled_bus_clear_fail_count;
    if(!wire_failure)wire_failure=PERIPH_FAIL_DATA_LOW;
    return 0u;
}
static uint32_t put_byte(uint8_t value)
{
    uint32_t i,ack;
    for(i=0u;i<8u;i++){
        sda(value&0x80u);delay_bit();if(!scl_high())return 0u;
        scl_low();value=(uint8_t)(value<<1);
    }
    sda(1u);delay_bit();if(!scl_high())return 0u;
    ack=!(GPIOA->IDR&SDA_PIN);scl_low();return ack && in_budget();
}
static uint32_t get_byte(uint8_t *value,uint32_t last)
{
    uint32_t i;uint8_t v=0u;sda(1u);
    for(i=0u;i<8u;i++){
        delay_bit();if(!scl_high())return 0u;
        v=(uint8_t)((v<<1)|((GPIOA->IDR&SDA_PIN)?1u:0u));scl_low();
    }
    sda(last);delay_bit();if(!scl_high())return 0u;
    scl_low();sda(1u);*value=v;return in_budget();
}
static uint32_t transfer(uint8_t address,const uint8_t *tx,uint32_t txlen,uint8_t *rx,uint32_t rxlen)
{
    uint32_t i,ok=0u;
    scl_pin=address==0x29u?(1u<<9):(1u<<4);
    sda_pin=address==0x29u?(1u<<10):(1u<<5);
    io_start=DWT->CYCCNT;wire_failure=0u;++peripheral_io_count;
    if(!idle_wire() || !start_wire())goto done;
    if(!put_byte((uint8_t)(address<<1))){if(!wire_failure)wire_failure=PERIPH_FAIL_ADDRESS_NACK;goto done;}
    for(i=0u;i<txlen;i++)if(!put_byte(tx[i])){if(!wire_failure)wire_failure=PERIPH_FAIL_DATA_NACK;goto done;}
    if(rxlen){
        if(!start_wire())goto done;
        if(!put_byte((uint8_t)((address<<1)|1u))){if(!wire_failure)wire_failure=PERIPH_FAIL_ADDRESS_NACK;goto done;}
        for(i=0u;i<rxlen;i++)if(!get_byte(&rx[i],i+1u==rxlen))goto done;
    }
    ok=1u;
 done:
    return finish_transfer(address,address==0x29u && txlen>=2u?((uint32_t)tx[0]<<8)|tx[1]:0xffffu,ok);
}
#endif
static uint32_t read_reg(uint16_t reg,uint8_t *data,uint32_t count)
{uint8_t b[2];b[0]=(uint8_t)(reg>>8);b[1]=(uint8_t)reg;return transfer(0x29u,b,2u,data,count);}
static uint32_t write_reg(uint16_t reg,const uint8_t *data,uint32_t count)
{uint8_t b[6];b[0]=(uint8_t)(reg>>8);b[1]=(uint8_t)reg;if(count>4u)return 0u;memcpy(b+2,data,count);return transfer(0x29u,b,count+2u,0,0u);}
static uint32_t write_byte(uint16_t reg,uint8_t data){return write_reg(reg,&data,1u);}
static uint32_t write_word(uint16_t reg,uint16_t data)
{uint8_t b[2];b[0]=(uint8_t)(data>>8);b[1]=(uint8_t)data;return write_reg(reg,b,2u);}
static void tof_fail(uint32_t error,uint32_t now)
{
    tof_io_failures=0u;
    tof_valid=0u;tof_error=error;++tof_error_count;
    tof_state=error==2u?TOF_WRONG_ID:TOF_ERROR;tof_phase=0u;tof_due=now+2000u;
}
/* Each call performs zero or one bounded bus transaction. */
static uint32_t service_tof(uint32_t now)
{
    uint8_t b[4];uint32_t n,value,ok=1u;
    if((int32_t)(now-tof_due)<0)return 0u;
    switch(tof_phase){
    case 0:
        tof_state=TOF_WAIT;tof_deadline=now+500u;tof_phase=1u;
        /* fall through */
    case 1:
        ok=read_reg(0x00e5u,b,1u);
        if(ok)tof_boot_readback=b[0];
        if(ok && (b[0]&1u)){tof_phase=2u;tof_due=now+1u;}
        else if(ok && (int32_t)(now-tof_deadline)>=0){tof_fail(3u,now);return 1u;}
        else if(ok)tof_due=now+5u;
        break;
    case 2:
        ok=read_reg(0x010fu,b,2u);
        if(ok){
            tof_sensor_id=((uint32_t)b[0]<<8)|b[1];
            if(tof_sensor_id!=0xeaccu){tof_fail(2u,now);return 1u;}
            tof_phase=3u;tof_state=TOF_CONFIG;
        }break;
    /* Use ST ULD's stop/configure/calibrate flow. Never assert SOFT_RESET:
     * a debugger/main reset between assert/release can strand the sensor. */
    case 3:ok=write_byte(0x0087u,0u);if(ok){config_offset=0u;tof_phase=4u;}break;
    case 4:
        n=91u-config_offset;if(n>4u)n=4u;
        ok=write_reg((uint16_t)(0x002du+config_offset),tof_boot_config+config_offset,n);
        if(ok){config_offset=(uint16_t)(config_offset+n);if(config_offset==91u)tof_phase=20u;}break;
    case 20:ok=read_reg(0x0030u,b,1u);if(ok){tof_mux_readback=b[0];tof_phase=5u;}break;
    case 5:
        ok=write_byte(0x0087u,0x40u);
        if(ok){tof_state=TOF_CALIBRATING;tof_phase=6u;tof_deadline=now+500u;tof_due=now+5u;}break;
    case 6:
        ok=read_reg(0x0031u,b,1u);
        if(ok)tof_ready_raw=b[0];
        if(ok && (b[0]&1u))tof_phase=7u;
        else if(ok && (int32_t)(now-tof_deadline)>=0){tof_fail(3u,now);return 1u;}
        else if(ok)tof_due=now+5u;
        break;
    case 7:ok=write_byte(0x0086u,1u);if(ok)tof_phase=8u;break;
    case 8:ok=write_byte(0x0087u,0u);if(ok)tof_phase=9u;break;
    case 9:ok=write_byte(0x0008u,9u);if(ok)tof_phase=10u;break;
    case 10:ok=write_byte(0x000bu,0u);if(ok)tof_phase=11u;break;
    case 11:ok=write_word(0x005eu,0x00adu);if(ok)tof_phase=12u;break;
    case 12:ok=write_word(0x0061u,0x00c6u);if(ok)tof_phase=13u;break;
    case 13:
        ok=read_reg(0x00deu,b,2u);
        if(ok){clock_pll=(uint16_t)((((uint16_t)b[0]<<8)|b[1])&0x03ffu);if(!clock_pll){tof_fail(4u,now);return 1u;}tof_phase=14u;}break;
    case 14:
        value=(uint32_t)clock_pll*TOF_PERIOD_MS*1075u/1000u;
        b[0]=(uint8_t)(value>>24);b[1]=(uint8_t)(value>>16);b[2]=(uint8_t)(value>>8);b[3]=(uint8_t)value;
        ok=write_reg(0x006cu,b,4u);if(ok)tof_phase=15u;break;
    case 15:
        ok=write_byte(0x0087u,0x40u);
        if(ok){tof_state=TOF_RUNNING;tof_error=0u;tof_phase=21u;tof_deadline=now+500u;tof_due=now+5u;}break;
    case 21:ok=read_reg(0x0087u,b,1u);if(ok){tof_start_readback=b[0];tof_phase=16u;}break;
    case 16:
        ok=read_reg(0x0031u,b,1u);
        if(ok)tof_ready_raw=b[0];
        if(ok && (b[0]&1u))tof_phase=17u;
        else if(ok && (int32_t)(now-tof_deadline)>=0){tof_fail(3u,now);return 1u;}
        else if(ok)tof_due=now+5u;
        break;
    case 17:ok=read_reg(0x0089u,b,1u);if(ok){raw_status=b[0]&0x1fu;tof_phase=22u;}break;
    case 22:ok=read_reg(0x008bu,b,1u);if(ok){tof_stream_count=b[0];tof_phase=18u;}break;
    case 18:ok=read_reg(0x0096u,b,2u);if(ok){pending_mm=(uint16_t)(((uint16_t)b[0]<<8)|b[1]);tof_phase=19u;}break;
    case 19:
        ok=write_byte(0x0086u,1u);
        if(ok){
            tof_range_status=raw_status;tof_range_mm=pending_mm;tof_valid=raw_status==9u && pending_mm>0u;
            tof_last_ms=now;++tof_sample_count;tof_phase=16u;tof_deadline=now+500u;tof_due=now+5u;
        }break;
    default:tof_fail(4u,now);return 0u;
    }
    if(!ok){
        if(++tof_io_failures<3u){++peripheral_io_retry_count;tof_due=now+2u;}
        else tof_fail(1u,now);
    }else tof_io_failures=0u;
    return 1u;
}

static void mark_byte(uint32_t index)
{
    uint32_t chunk=index/4u;uint8_t bit=(uint8_t)(1u<<(chunk&7u));
    if(!(dirty[chunk/8u]&bit)){dirty[chunk/8u]|=bit;++oled_dirty_chunks;}
}
static void pixel(uint8_t *target,uint32_t page0,uint32_t x,uint32_t y)
{
    if(x>=128u || y>=64u || y/8u<page0)return;
    target[(y/8u-page0)*128u+x]|=(uint8_t)(1u<<(y&7u));
}
static const uint8_t digit_font[10][5]={
 {0x3e,0x51,0x49,0x45,0x3e},{0,0x42,0x7f,0x40,0},{0x42,0x61,0x51,0x49,0x46},
 {0x21,0x41,0x45,0x4b,0x31},{0x18,0x14,0x12,0x7f,0x10},{0x27,0x45,0x45,0x45,0x39},
 {0x3c,0x4a,0x49,0x49,0x30},{1,0x71,9,5,3},{0x36,0x49,0x49,0x49,0x36},{6,0x49,0x49,0x29,0x1e}
};
static void glyph(char c,uint8_t out[5])
{
    static const char letters[]="DISTANCECM.";
    static const uint8_t font[][5]={
      {0x7f,0x41,0x41,0x22,0x1c},{0,0x41,0x7f,0x41,0},{0x26,0x49,0x49,0x49,0x32},
      {1,1,0x7f,1,1},{0x7e,0x11,0x11,0x11,0x7e},{0x7f,2,4,8,0x7f},
      {0x3e,0x41,0x41,0x41,0x22},{0x7f,0x49,0x49,0x49,0x41},
      {0x3e,0x41,0x41,0x41,0x22},{0x7f,2,0x0c,2,0x7f},{0,0x60,0x60,0,0}
    };
    uint32_t i;memset(out,0,5u);
    if(c>='0' && c<='9'){memcpy(out,digit_font[(uint32_t)(c-'0')],5u);return;}
    for(i=0u;i<sizeof(letters)-1u;i++)if(letters[i]==c){memcpy(out,font[i],5u);return;}
}
static void text(uint8_t *target,uint32_t page0,uint32_t x,uint32_t y,const char *s,uint32_t scale)
{
    uint8_t g[5];uint32_t col,row,dx,dy;
    while(*s){glyph(*s++,g);
        for(col=0u;col<5u;col++)for(row=0u;row<7u;row++)if(g[col]&(1u<<row))
            for(dx=0u;dx<scale;dx++)for(dy=0u;dy<scale;dy++)pixel(target,page0,x+col*scale+dx,y+row*scale+dy);
        x+=6u*scale;
    }
}
static void render_distance(uint32_t mm)
{
    char value[10];uint32_t i,n=0u,cm=mm/10u;
    memset(number_frame,0,sizeof(number_frame));
    if(cm>=100u)value[n++]=(char)('0'+cm/100u);
    if(cm>=10u)value[n++]=(char)('0'+(cm/10u)%10u);
    value[n++]=(char)('0'+cm%10u);value[n++]='.';value[n++]=(char)('0'+mm%10u);
    value[n++]=' ';value[n++]='C';value[n++]='M';value[n]=0;
    text(number_frame,3u,(128u-12u*n)/2u,28u,value,2u);
    /* Compare final pixels, not the temporary clear-and-repaint steps. */
    for(i=0u;i<sizeof(number_frame);i++)if(frame[384u+i]!=number_frame[i]){
        frame[384u+i]=number_frame[i];mark_byte(384u+i);
    }
    ++oled_render_count;
}
static int32_t next_dirty(void)
{
    uint32_t i,j,index;for(i=0u;i<32u;i++){
        index=(oled_scan_byte+i)&31u;if(!dirty[index])continue;
        for(j=0u;j<8u;j++)if(dirty[index]&(1u<<j))return (int32_t)(index*8u+j);
    }return -1;
}
static uint32_t oled_write(uint8_t control,const uint8_t *data,uint32_t count)
{uint8_t b[5];uint32_t ok;b[0]=control;if(count>4u)return 0u;memcpy(b+1,data,count);ok=transfer((uint8_t)oled_address,b,count+1u,0,0u);
 /* After a failed pixel chunk, a successful position command is only the
  * first half of recovery. Clear its failure streak only when pixels succeed. */
 if(ok && (control==0x40u || !oled_pixel_retry)){oled_io_failures=0u;oled_pixel_retry=0u;}return ok;}
static void oled_fail(uint32_t now)
{
    ++oled_error_count;oled_cursor_valid=0u;oled_data_pending=0u;
    /* A partial pixel transfer may have advanced the controller cursor.
     * Retry from a fresh position command, leaving the dirty chunk marked. */
    if(oled_state==OLED_READY && ++oled_io_failures<3u){++peripheral_io_retry_count;oled_due=now+2u;return;}
    oled_io_failures=0u;oled_pixel_retry=0u;oled_state=OLED_ERROR;oled_phase=0u;oled_due=now+2000u;
}
static uint32_t flush_oled_data(uint32_t now)
{
    uint32_t ok=oled_write(0x40u,frame+oled_pending_chunk*4u,4u);
    if(ok){
        dirty[oled_pending_chunk/8u]&=(uint8_t)~(1u<<(oled_pending_chunk&7u));
        --oled_dirty_chunks;
        oled_scan_byte=(uint8_t)(oled_pending_chunk/8u);oled_cursor=(uint16_t)(oled_pending_chunk*4u+4u);
        if((oled_cursor&127u)==0u)oled_cursor_valid=0u;oled_data_pending=0u;
    }else{oled_pixel_retry=1u;oled_fail(now);}return 1u;
}
static uint32_t service_oled(uint32_t now,uint32_t visible)
{
    uint8_t b[3];uint32_t ok;int32_t chunk;
    if((int32_t)(now-oled_due)<0)return 0u;
    if(oled_phase<2u){
        oled_address=oled_phase==0u?0x3cu:0x3du;
        ok=transfer((uint8_t)oled_address,0,0u,0,0u);
        if(ok){oled_phase=2u;oled_state=OLED_INIT;oled_init_step=0u;oled_io_failures=0u;}
        else if(oled_phase==0u)oled_phase=1u;else oled_fail(now);
        return 1u;
    }
    if(oled_phase==2u){
        ok=oled_write(0u,oled_commands[oled_init_step],oled_command_lengths[oled_init_step]);
        if(ok){
            if(oled_init_step==0u)oled_on=0u;
            if(++oled_init_step==sizeof(oled_command_lengths)){
                oled_state=OLED_READY;oled_phase=3u;memset(dirty,0xff,sizeof(dirty));oled_dirty_chunks=256u;oled_cursor_valid=0u;
            }
        }else oled_fail(now);return 1u;
    }
    if(oled_on && !visible){b[0]=0xaeu;ok=oled_write(0u,b,1u);if(ok)oled_on=0u;else oled_fail(now);return 1u;}
    if(oled_data_pending)return flush_oled_data(now);
    chunk=next_dirty();
    if(chunk>=0){
        oled_pending_chunk=(uint16_t)chunk;oled_data_pending=1u;
        if(!oled_cursor_valid || oled_cursor!=(uint16_t)(chunk*4)){
            b[0]=(uint8_t)(0xb0u+(uint32_t)chunk/32u);b[1]=(uint8_t)(((uint32_t)chunk*4u)&15u);
            b[2]=(uint8_t)(0x10u+(((uint32_t)chunk*4u)&127u)/16u);
            ok=oled_write(0u,b,3u);if(ok){oled_cursor_valid=1u;oled_cursor=(uint16_t)(chunk*4);}else oled_fail(now);
            return 1u;
        }
        /* A contiguous four-byte data write consumes this call's only I/O. */
        return flush_oled_data(now);
    }
    if(!oled_on && visible && rendered_mm!=0xffffffffu){b[0]=0xafu;ok=oled_write(0u,b,1u);if(ok)oled_on=1u;else oled_fail(now);return 1u;}
    return 0u;
}
void peripherals_init(void)
{
    uint32_t now=uptime_ms;
#ifndef PERIPHERAL_SELFTEST
    RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;
    GPIOA->BSRR=(1u<<4)|(1u<<5)|(1u<<9)|(1u<<10);GPIOA->BRR=1u<<7;
    GPIOA->CRL=(GPIOA->CRL&~(0xffu<<16))|(0x66u<<16); /* PA4/5 open drain. */
    GPIOA->CRH=(GPIOA->CRH&~(0xffu<<4))|(0x66u<<4); /* PA9/10 FT open drain; preserve PA8 STBY. */
    GPIOA->CRL=(GPIOA->CRL&~(0xfu<<28))|(0x2u<<28);  /* PA7 optional indicator. */
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CTRL|=1u;
#endif
    tof_state=TOF_WAIT;tof_sensor_id=0u;tof_range_mm=0u;tof_range_status=255u;tof_valid=0u;
    tof_sample_count=0u;tof_last_ms=0u;tof_error=0u;tof_error_count=0u;
    oled_state=OLED_WAIT;oled_address=0u;oled_on=0u;oled_error_count=0u;
    peripheral_io_count=0u;peripheral_io_max_us=0u;obstacle_indicator_on=0u;
    tof_mux_readback=0u;tof_start_readback=0u;tof_stream_count=0u;tof_ready_raw=0u;
    peripheral_timeout_count=0u;peripheral_fail_address=0u;peripheral_fail_reg=0u;
    tof_reset_count=0u;tof_boot_readback=0u;
    peripheral_io_retry_count=0u;tof_io_failures=0u;oled_io_failures=0u;last_io_tof=0u;
    peripheral_bus_clear_count=0u;peripheral_bus_clear_fail_count=0u;peripheral_fail_kind=0u;
    tof_bus_clear_count=0u;tof_bus_clear_fail_count=0u;tof_transfer_error_count=0u;tof_timeout_count=0u;tof_fail_kind=0u;tof_io_max_us=0u;
    oled_bus_clear_count=0u;oled_bus_clear_fail_count=0u;oled_transfer_error_count=0u;oled_timeout_count=0u;oled_fail_kind=0u;oled_io_max_us=0u;
    oled_dirty_chunks=256u;oled_render_count=0u;
    tof_due=now+200u;oled_due=now+200u;tof_phase=0u;oled_phase=0u;config_offset=0u;
    last_poll=now-1u;render_ms=now-OLED_REFRESH_MS;rendered_mm=0xffffffffu;
    oled_scan_byte=0u;oled_data_pending=0u;oled_pixel_retry=0u;oled_cursor_valid=0u;memset(frame,0,sizeof(frame));memset(dirty,0xff,sizeof(dirty));
    text(frame,0u,40u,8u,"DISTANCE",1u);
}
void peripherals_poll(void)
{
    uint32_t now=uptime_ms,visible,did_io;
    if(now==last_poll)return;last_poll=now;
    /* Main already runs IMU and balance before this call. Do not reject
     * peripherals solely because a 100 Hz IMU sample is over 5 ms old: on a
     * loaded loop this can starve both buses. Keep one bounded I/O per ms;
     * the original motor/IMU validity guards and interrupt stop remain. */
    if(tof_valid && (uint32_t)(now-tof_last_ms)>TOF_STALE_MS)tof_valid=0u;
    visible=tof_valid && tof_range_mm>0u && tof_range_mm<=OBSTACLE_SHOW_MM;
    /* Finish a coherent image before accepting the next one. While off,
     * compose only the first recovered image; then let AF run before another
     * changing range can refill the queue. Control still runs first. */
    if(visible && !oled_dirty_chunks && !oled_data_pending &&
       (oled_on || rendered_mm==0xffffffffu) &&
       (rendered_mm==0xffffffffu || (tof_range_mm!=rendered_mm && (uint32_t)(now-render_ms)>=OLED_REFRESH_MS))){
        render_distance(tof_range_mm);rendered_mm=tof_range_mm;render_ms=now;
    }
    if(!visible)rendered_mm=0xffffffffu;
    obstacle_indicator_on=visible;
#ifndef PERIPHERAL_SELFTEST
    if(visible)GPIOA->BSRR=1u<<7;else GPIOA->BRR=1u<<7;
#endif
    /* When a loaded loop visits every >=5 ms, a ready-check transaction can
     * otherwise take every slot forever. Alternate successful service slots;
     * an idle OLED yields immediately, so range polling keeps its priority. */
    if(last_io_tof && service_oled(now,visible)){last_io_tof=0u;return;}
    did_io=service_tof(now);
    if(did_io)last_io_tof=1u;
    else if(service_oled(now,visible))last_io_tof=0u;
}
#ifdef PERIPHERAL_SELFTEST
#include "selftest.h"
extern volatile uint32_t selftest_failure_lines[32];
static void pcheck(uint32_t passed,uint32_t line)
{++selftest_checks;if(!passed){if(selftest_failures<32u)selftest_failure_lines[selftest_failures]=90000u+line;++selftest_failures;}}
#define PCHECK(x) pcheck((x),__LINE__)
static void sim_reset(uint32_t present)
{
    pd_window_cancel();uptime_ms=0u;sim_present=present;sim_wrong_id=0u;sim_never_ready=0u;
    sim_bad_status=0u;sim_distance=150u;sim_config_writes=0u;sim_alt_oled=0u;sim_bus_fail=0u;
    sim_scheduler_overrun=0u;sim_boot_at=0u;sim_preboot_config=0u;sim_reset_writes=0u;sim_tof_nacks=0u;sim_oled_nacks=0u;sim_oled_data_nacks=0u;peripherals_init();
}
static void run_ms(uint32_t ms)
{
    uint32_t i,before;for(i=0u;i<ms;i++){
        ++uptime_ms;before=peripheral_io_count;peripherals_poll();
        if(peripheral_io_count-before>1u)sim_scheduler_overrun=1u;
    }
}
void peripherals_selftest_run(void)
{
    uint32_t before,i,errors;
    io_start=0u;wire_failure=0u;sim_wire_ticks=0u;sim_clock_reads=0u;sim_clock_mode=0u;
    PCHECK(scl_high() && wire_failure==0u);
    io_start=0u;wire_failure=0u;sim_wire_ticks=0u;sim_clock_reads=0u;sim_clock_mode=1u;
    PCHECK(scl_high() && wire_failure==0u && sim_clock_reads>=2u);
    io_start=0u;wire_failure=0u;sim_wire_ticks=0u;sim_clock_reads=0u;sim_clock_mode=2u;
    PCHECK(!scl_high() && wire_failure==PERIPH_FAIL_CLOCK_LOW && sim_wire_ticks<=800u);
    io_start=0u;wire_failure=0u;sim_wire_ticks=0u;sim_clock_reads=0u;sim_clock_mode=3u;
    PCHECK(!scl_high() && wire_failure==PERIPH_FAIL_DEADLINE);
    sim_clock_mode=0u;
    sim_reset(3u);run_ms(150u);PCHECK(peripheral_io_count==0u && !oled_on);
    run_ms(1350u);PCHECK(tof_sensor_id==0xeaccu && tof_state==TOF_RUNNING && tof_valid && tof_range_mm==150u && tof_sample_count>5u);
    PCHECK(sim_reset_writes==0u && tof_boot_readback==1u && !sim_preboot_config && tof_error_count==0u);
    PCHECK(oled_state==OLED_READY && oled_on && obstacle_indicator_on);
    PCHECK(!sim_scheduler_overrun);
    before=tof_reset_count;sim_tof_nacks=1u;run_ms(250u);
    PCHECK(tof_reset_count==before && tof_state==TOF_RUNNING && tof_valid && peripheral_io_retry_count>=1u);
    before=oled_error_count;sim_oled_nacks=1u;sim_distance=151u;run_ms(500u);
    PCHECK(oled_state==OLED_READY && oled_on && oled_error_count==before+1u && peripheral_io_retry_count>=2u);
    sim_tof_nacks=3u;run_ms(80u);PCHECK(tof_state==TOF_ERROR && !tof_valid && !oled_on);
    run_ms(2300u);PCHECK(tof_state==TOF_RUNNING && tof_valid && oled_on);sim_distance=150u;
    sim_distance=2501u;run_ms(300u);PCHECK(tof_valid && !oled_on && !obstacle_indicator_on);
    sim_distance=150u;run_ms(500u);PCHECK(oled_on && obstacle_indicator_on);
    sim_bad_status=1u;run_ms(250u);PCHECK(!tof_valid && !oled_on && !obstacle_indicator_on);
    sim_bad_status=0u;sim_distance=0u;run_ms(200u);PCHECK(!tof_valid && !oled_on);
    sim_distance=150u;run_ms(400u);sim_never_ready=1u;run_ms(350u);PCHECK(!tof_valid && !oled_on && !obstacle_indicator_on);
    sim_reset(2u);run_ms(1500u);PCHECK(!tof_valid && tof_state==TOF_ERROR && oled_state==OLED_READY && !oled_on);
    sim_reset(3u);sim_wrong_id=1u;run_ms(1000u);PCHECK(tof_state==TOF_WRONG_ID && sim_config_writes==0u);
    sim_reset(3u);sim_never_ready=1u;run_ms(1500u);PCHECK(tof_state==TOF_ERROR && tof_error==3u && !oled_on);
    sim_reset(1u);run_ms(1500u);PCHECK(tof_valid && oled_state==OLED_ERROR && !oled_on);
    sim_reset(0u);run_ms(1500u);PCHECK(!tof_valid && !oled_on && !sim_scheduler_overrun && !observed_stby);
    sim_reset(3u);sim_alt_oled=1u;run_ms(1500u);PCHECK(oled_address==0x3du && oled_on);
    pd_window_active=1u;imu_valid=1u;imu_last_sample_ms=uptime_ms-8u;before=peripheral_io_count;
    run_ms(10u);PCHECK(peripheral_io_count>before);
    pd_window_active=0u;sim_bus_fail=1u;run_ms(50u);PCHECK(!tof_valid && !sim_scheduler_overrun);
    sim_bus_fail=0u;peripherals_init();PCHECK(tof_sample_count==0u && !tof_valid && !oled_on && !obstacle_indicator_on);
    sim_reset(3u);sim_boot_at=350u;run_ms(1500u);
    PCHECK(tof_valid && !sim_preboot_config && !sim_reset_writes && !tof_error_count);
    /* Exercise the active-session scheduler at the old starvation boundary. */
    sim_reset(3u);run_ms(1500u);pd_window_active=1u;imu_valid=1u;
    before=tof_sample_count;
    for(i=0u;i<1200u;i++){imu_last_sample_ms=uptime_ms-8u;run_ms(1u);}
    PCHECK(tof_sample_count>=before+10u && tof_valid && oled_on && !sim_scheduler_overrun);
    pd_window_active=0u;before=tof_sample_count;run_ms(250u);
    PCHECK(tof_sample_count>=before+2u && tof_valid && oled_on);
    /* Position ACKs must not hide consecutive failed pixel chunks. */
    errors=oled_error_count;sim_oled_data_nacks=2u;sim_distance=152u;
    render_ms=uptime_ms-OLED_REFRESH_MS;run_ms(150u);
    PCHECK(oled_error_count==errors+2u && oled_state==OLED_READY && oled_on && !oled_io_failures);
    sim_oled_data_nacks=3u;sim_distance=154u;
    render_ms=uptime_ms-OLED_REFRESH_MS;
    /* Fair sharing changes the phase of the next range sample. Wait for the
     * injected error event within two range periods plus retry overhead. */
    for(i=0u;i<250u && oled_state!=OLED_ERROR;i++)run_ms(1u);
    PCHECK(oled_state==OLED_ERROR && !sim_oled_data_nacks);
    run_ms(2400u);PCHECK(oled_state==OLED_READY && oled_on && tof_valid && !sim_scheduler_overrun);
    sim_oled_nacks=1u;sim_distance=2501u;run_ms(300u);
    PCHECK(!oled_on && oled_state==OLED_READY && !oled_io_failures && !oled_pixel_retry);
    pd_window_cancel();
}
#ifdef REVIEW_ONLY_SELFTEST
void peripherals_review_selftest_run(void)
{
    uint32_t i;
    /* Run the actual STOP/finalization with injected clock failure. */
    sim_reset(3u);io_start=0u;sim_wire_ticks=0u;sim_clock_reads=0u;sim_clock_mode=2u;wire_failure=0u;
    PCHECK(!finish_transfer(0x3cu,0xffffu,1u));
    PCHECK(oled_transfer_error_count==1u && oled_fail_kind==PERIPH_FAIL_CLOCK_LOW && !tof_transfer_error_count);
    PCHECK(!peripheral_timeout_count && !oled_timeout_count);
    io_start=0u;sim_wire_ticks=0u;sim_clock_reads=0u;sim_clock_mode=3u;wire_failure=0u;
    PCHECK(!finish_transfer(0x29u,0x0096u,1u));
    PCHECK(tof_transfer_error_count==1u && tof_timeout_count==1u && tof_fail_kind==PERIPH_FAIL_DEADLINE && peripheral_fail_address==0x29u);
    io_start=0u;sim_wire_ticks=0u;sim_clock_reads=0u;sim_clock_mode=0u;wire_failure=0u;
    PCHECK(finish_transfer(0x3cu,0xffffu,1u));
    /* A stable final image must not be dirtied just by clearing/repainting. */
    sim_reset(3u);run_ms(1500u);PCHECK(next_dirty()<0);
    render_distance(150u);PCHECK(next_dirty()<0);
    run_ms(500u);
    /* Model a loaded main loop: one opportunity every 10 ms, with changing
     * valid distance after a deliberate off transition. No bus faults. */
    sim_distance=2501u;run_ms(300u);PCHECK(!oled_on);
    pd_window_active=1u;
    for(i=0u;i<500u;i++){
        uptime_ms+=10u;sim_distance=150u+i%3u;peripherals_poll();
    }
    PCHECK(tof_valid && oled_on && !oled_error_count && !tof_error_count);
    sim_bad_status=1u;run_ms(250u);PCHECK(!oled_on);
    sim_bad_status=0u;
    for(i=0u;i<500u;i++){
        uptime_ms+=10u;sim_distance=250u+i%3u;peripherals_poll();
    }
    PCHECK(tof_valid && oled_on && !sim_scheduler_overrun);
    pd_window_cancel();
}
#endif
#endif
#endif

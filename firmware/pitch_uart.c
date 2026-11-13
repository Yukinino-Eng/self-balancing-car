#include "pitch_uart.h"
#include "board.h"
#include "runtime.h"
#include "imu.h"

#define PITCH_PERIOD_MS 50u
#define PITCH_DMA_TIMEOUT_MS 10u
#define DMA7_CLEAR (15u<<24)
#define DMA7_COMPLETE (1u<<25)
#define DMA7_ERROR (1u<<27)
volatile uint32_t pitch_uart_state,pitch_uart_busy;
volatile uint32_t pitch_uart_started,pitch_uart_completed;
volatile uint32_t pitch_uart_errors;
char pitch_uart_buffer[17];
static uint32_t next_ms,started_ms;
#ifdef LEARN_SIMULATOR
uint32_t pitch_uart_sim_result;
#endif

uint32_t pitch_uart_format(char out[17],float pitch,uint32_t valid)
{
    const char prefix[]="PITCH=";
    const char invalid[]="PITCH=INVALID\r\n";
    uint32_t i,value,negative;
    if(!valid || !(pitch>=-180.0f && pitch<=180.0f)){
        for(i=0u;i<sizeof(invalid);i++)out[i]=invalid[i];
        return sizeof(invalid)-1u;
    }
    negative=pitch<0.0f;
    value=(uint32_t)((negative?-pitch:pitch)*1000.0f+0.5f);
    for(i=0u;i<6u;i++)out[i]=prefix[i];
    out[6]=negative && value ? '-' : '+';
    out[7]=(char)('0'+value/100000u);
    out[8]=(char)('0'+value/10000u%10u);
    out[9]=(char)('0'+value/1000u%10u);
    out[10]='.';
    out[11]=(char)('0'+value/100u%10u);
    out[12]=(char)('0'+value/10u%10u);
    out[13]=(char)('0'+value%10u);
    out[14]='\r';out[15]='\n';out[16]=0;
    return 16u;
}
static void dma_stop(void)
{
#ifndef LEARN_SIMULATOR
    DMA1_Channel7->CCR=0u;
    DMA1->IFCR=DMA7_CLEAR;
#else
    pitch_uart_sim_result=0u;
#endif
    pitch_uart_busy=0u;
}
static void fail(void)
{
    dma_stop();++pitch_uart_errors;pitch_uart_state=PITCH_UART_FAILED;
#ifndef LEARN_SIMULATOR
    USART2->CR3=0u;
#endif
}
void pitch_uart_init(void)
{
    pitch_uart_state=PITCH_UART_OFF;pitch_uart_busy=0u;
    pitch_uart_started=0u;pitch_uart_completed=0u;
    pitch_uart_errors=0u;
    next_ms=uptime_ms;started_ms=uptime_ms;
#ifndef LEARN_SIMULATOR
    /* board_init selects 8MHz HSI with HCLK=PCLK1=8MHz. Never steal an
     * active UART/DMA channel or silently transmit at an incorrect rate. */
    if((RCC->CFGR & (RCC_CFGR_SWS|RCC_CFGR_HPRE|RCC_CFGR_PPRE1))!=0u ||
       (AFIO->MAPR & AFIO_MAPR_USART2_REMAP)!=0u){
        ++pitch_uart_errors;pitch_uart_state=PITCH_UART_FAILED;return;
    }
    RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR|=RCC_APB1ENR_USART2EN;
    RCC->AHBENR|=RCC_AHBENR_DMA1EN;
    if((USART2->CR1 & USART_CR1_UE)!=0u || (DMA1_Channel7->CCR & 1u)!=0u){
        ++pitch_uart_errors;pitch_uart_state=PITCH_UART_FAILED;return;
    }
    GPIOA->CRL=(GPIOA->CRL & ~(15u<<8)) | (10u<<8); /* PA2 AF push-pull 2MHz. */
    USART2->CR1=0u;USART2->CR2=0u;
    USART2->BRR=(8000000u+57600u)/115200u;
    USART2->CR3=USART_CR3_DMAT;
    DMA1_Channel7->CCR=0u;DMA1->IFCR=DMA7_CLEAR;
    DMA1_Channel7->CPAR=(uint32_t)&USART2->DR;
    DMA1_Channel7->CMAR=(uint32_t)pitch_uart_buffer;
    USART2->CR1=USART_CR1_UE|USART_CR1_TE;
#else
    pitch_uart_sim_result=0u;
#endif
    pitch_uart_state=PITCH_UART_READY;
}
void pitch_uart_poll(void)
{
    uint32_t now=uptime_ms,complete,error,length,valid;
    if(pitch_uart_state!=PITCH_UART_READY)return;
    if(pitch_uart_busy){
#ifndef LEARN_SIMULATOR
        complete=DMA1->ISR & DMA7_COMPLETE;
        error=DMA1->ISR & DMA7_ERROR;
#else
        complete=pitch_uart_sim_result==1u;
        error=pitch_uart_sim_result==2u;
#endif
        if(error){fail();return;}
        if(complete){dma_stop();++pitch_uart_completed;}
        else if((uint32_t)(now-started_ms)>=PITCH_DMA_TIMEOUT_MS){fail();return;}
    }
    if((int32_t)(now-next_ms)<0)return;
    next_ms=now+PITCH_PERIOD_MS; /* No catch-up bursts after a delayed main loop. */
    if(pitch_uart_busy)return;
    valid=imu_valid && imu_fusion_valid && imu_sample_count && !imu_frozen &&
          !(imu_seq&1u) && (uint32_t)(now-imu_last_sample_ms)<=30u;
    length=pitch_uart_format(pitch_uart_buffer,imu_body_pitch_deg,valid);
    started_ms=now;
#ifndef LEARN_SIMULATOR
    DMA1_Channel7->CNDTR=length;
    __DMB();
    DMA1_Channel7->CCR=(1u<<7)|(1u<<4)|1u; /* Increment memory, byte TX, low priority. */
#else
    (void)length;
#endif
    pitch_uart_busy=1u;++pitch_uart_started;
}

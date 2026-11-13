#include "encoder.h"

volatile int32_t encoder_e1_count = 0;
volatile int32_t encoder_e2_count = 0;
volatile int32_t encoder_e1_step = 0;
volatile int32_t encoder_e2_step = 0;
volatile uint32_t encoder_sample_count = 0u;

#ifdef LEARN_SIMULATOR
volatile uint32_t encoder_sim_e1_raw = 0u;
volatile uint32_t encoder_sim_e2_raw = 0u;
#endif

static uint32_t previous_e1;
static uint32_t previous_e2;
static uint32_t accumulated_e1;
static uint32_t accumulated_e2;

int32_t encoder_delta16(uint32_t current, uint32_t previous)
{
    uint32_t bits = (current - previous) & 0xFFFFu;
    /* Each 1 ms interval must contain fewer than 32768 edges. */
    return bits < 0x8000u ? (int32_t)bits : (int32_t)bits - 65536;
}

int32_t encoder_signed32(uint32_t bits)
{
    /* Defined modulo arithmetic, without overflowing a signed int32_t. */
    return bits <= 0x7FFFFFFFu ? (int32_t)bits :
           -1 - (int32_t)(0xFFFFFFFFu - bits);
}

#ifndef LEARN_SIMULATOR
static void configure_encoder(TIM_TypeDef *timer)
{
    timer->CR1 = 0u;
    timer->SMCR = 3u;                /* TI12, quadrature x4. */
    timer->CCMR1 = 0x0101u;          /* TI1/TI2 inputs, no digital filter. */
    timer->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    timer->PSC = 0u;
    timer->ARR = 0xFFFFu;
    timer->CNT = 0u;
    timer->EGR = TIM_EGR_UG;
    timer->SR = 0u;
    timer->CR1 = TIM_CR1_CEN;
}
#endif

void encoder_init(void)
{
#ifndef LEARN_SIMULATOR
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;
    AFIO->MAPR &= ~((3u << 8) | (1u << 12)); /* Default TIM2/4 mapping. */
    GPIOA->CRL = (GPIOA->CRL & ~0xFFu) | 0x44u; /* PA0/1 E2 floating inputs. */
    GPIOB->CRL = (GPIOB->CRL & ~0xFF000000u) | 0x44000000u; /* PB6/7 E1. */
    configure_encoder(TIM4);
    configure_encoder(TIM2);
#else
    encoder_sim_e1_raw = 0u;
    encoder_sim_e2_raw = 0u;
#endif
    previous_e1 = 0u;
    previous_e2 = 0u;
    accumulated_e1 = 0u;
    accumulated_e2 = 0u;
    encoder_e1_count = 0;
    encoder_e2_count = 0;
    encoder_e1_step = 0;
    encoder_e2_step = 0;
    encoder_sample_count = 0u;
}

void encoder_sample(void)
{
    uint32_t current_e1;
    uint32_t current_e2;
#ifndef LEARN_SIMULATOR
    current_e1 = TIM4->CNT & 0xFFFFu;
    current_e2 = TIM2->CNT & 0xFFFFu;
#else
    current_e1 = encoder_sim_e1_raw & 0xFFFFu;
    current_e2 = encoder_sim_e2_raw & 0xFFFFu;
#endif
    encoder_e1_step = encoder_delta16(current_e1, previous_e1);
    encoder_e2_step = encoder_delta16(current_e2, previous_e2);
    previous_e1 = current_e1;
    previous_e2 = current_e2;
    accumulated_e1 += (uint32_t)encoder_e1_step;
    accumulated_e2 += (uint32_t)encoder_e2_step;
    encoder_e1_count = encoder_signed32(accumulated_e1);
    encoder_e2_count = encoder_signed32(accumulated_e2);
    ++encoder_sample_count;
}


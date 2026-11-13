#include "board.h"
#ifdef PD_HOLD_TEST
#include "pd_window.h"
volatile uint32_t hold_actuator_limited=0u;
#endif

#ifdef LEARN_SIMULATOR
const uint32_t build_is_simulator = 1u;
#else
const uint32_t build_is_simulator = 0u;
#endif

volatile uint32_t observed_stby = 0u;
volatile uint32_t observed_direction_pins = 0u;
volatile uint32_t observed_pwm_a = 0u;
volatile uint32_t observed_pwm_b = 0u;
volatile uint32_t watchdog_started = 0u;
volatile uint32_t observed_led_on = 0u;

/* Called before C variables are initialized. Do not use globals here. */
void SystemInit(void)
{
#ifndef LEARN_SIMULATOR
    /* Reset starts on the internal 8 MHz HSI oscillator. */
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0u) { }
    RCC->CFGR = 0u;                  /* HSI; AHB/APB prescalers all 1. */
    while ((RCC->CFGR & RCC_CFGR_SWS) != 0u) { }
    RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON | RCC_CR_CSSON);

    /* On a watchdog reset the IWDG is still running; refresh early. */
    IWDG->KR = 0xAAAAu;
#endif
    SCB->VTOR = 0x08000000u;
}

void board_motor_off(void)
{
#ifndef LEARN_SIMULATOR
    /* 先将 STBY 拉低关闭驱动，再把 PWM 和方向全部清零。 */
    GPIOA->BRR = MOTOR_STBY_PIN;
    TIM3->CCER = 0u;
    TIM3->CCR3 = 0u;
    TIM3->CCR4 = 0u;
    TIM3->EGR = TIM_EGR_UG;           /* Immediately commit zero preload. */
    GPIOB->BRR = MOTOR_DIRECTION_PINS;
#endif
#ifdef PD_HOLD_TEST
    hold_actuator_limited=0u;
    hold_sync_applied_counts=0;hold_sync_saturation_error_counts=0;
#endif
    observed_stby = 0u;
    observed_direction_pins = 0u;
    observed_pwm_a = 0u;
    observed_pwm_b = 0u;
}

void board_init(void)
{
#ifndef LEARN_SIMULATOR
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* Load low levels before switching from inputs to output pins. */
    GPIOA->BRR = MOTOR_STBY_PIN;
    GPIOB->BRR = MOTOR_DIRECTION_PINS | 3u;
    GPIOA->CRH = (GPIOA->CRH & ~0xFu) | 0x2u; /* PA8: push-pull. */
    GPIOB->CRH = (GPIOB->CRH & ~0xFFFF0000u) | 0x22220000u;
    GPIOC->BSRR = 1u << 13;
    GPIOC->CRH = (GPIOC->CRH & ~(0xFu << 20)) | (0x2u << 20);

    /* TIM3 定时器输出 PWM：PB0 控制电机 B，PB1 控制电机 A。 */
    AFIO->MAPR &= ~(3u << 10);
    TIM3->CR1 = 0u;
    TIM3->PSC = 0u;
    TIM3->ARR = PWM_PERIOD_COUNTS - 1u;
    TIM3->CCMR2 = 0x6868u;           /* CH3/4 PWM mode 1, preload. */
    board_motor_off();
    GPIOB->CRL = (GPIOB->CRL & ~0xFFu) | 0xAAu; /* AF push-pull. */
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    /* 800 万 / 400 = 2 万赫兹；占空比 10% 对应比较值 40。 */
    DBGMCU->CR &= ~(DBGMCU_CR_DBG_IWDG_STOP | DBGMCU_CR_DBG_TIM3_STOP);
#endif
    board_motor_off();
}

void board_tick_init(void)
{
    /* HSI 8 MHz / 8000 = 1 kHz: one interrupt per millisecond. */
    (void)SysTick_Config(8000u);
}

#ifdef PD_HOLD_TEST
volatile float hold_right_boost_percent = (float)HOLD_RIGHT_BOOST_PERMILLE / 10.0f;
volatile uint32_t hold_right_boost_counts = HOLD_RIGHT_BOOST_COUNTS;
volatile uint32_t hold_right_boost_max_base_percent = HOLD_RIGHT_BOOST_MAX_BASE_PERCENT;

volatile int32_t hold_sync_applied_counts,hold_sync_saturation_error_counts;

/* Hold-only dual-wheel writer: preserve ordinary and fixed-load APIs. */
/* Native 400-count PWM. Positive sync adds forward torque to the left. */
void board_hold_start_counts(uint32_t command, uint32_t base, int32_t sync)
{
    uint32_t pins, left_compare, right_compare, boost, handoff, starting, common, limited;
    uint32_t limit = command == 6u ? hold_backward_limit_percent : hold_output_limit_percent;
    uint32_t hard_limit = command == 6u ? HOLD_BACKWARD_LIMIT_PERCENT : HOLD_OUTPUT_LIMIT_PERCENT;
    int32_t low, high, half, requested_sync=sync, applied_sync;
    if ((command != 5u && command != 6u) || base == 0u ||
        limit == 0u || limit > hard_limit || base > limit*4u ||
        hold_right_boost_counts > 40u || hold_right_boost_max_base_percent > 100u ||
        hold_common_boost_counts > HOLD_MAX_COMMON_BOOST_COUNTS) {
        board_motor_off(); return;
    }
    pins = command == 5u ? (MOTOR_A1_PIN | MOTOR_B1_PIN) :
                          (MOTOR_A2_PIN | MOTOR_B2_PIN);
    /* Fade in trim over the first 2% base; retain monotone plateau handoff. */
    boost = base < 8u ? (base*hold_right_boost_counts+4u)/8u : hold_right_boost_counts;
    handoff = 4u*hold_right_boost_max_base_percent+hold_right_boost_counts;
    right_compare = base+boost;
    if (right_compare > handoff) right_compare=handoff;
    if (right_compare < base) right_compare=base;
    if (right_compare > limit*4u) right_compare=limit*4u;
    /* Continuous odd feedforward: extra = F*|u|/(|u|+1%).
       Both wheels receive the same extra before differential adjustment.
       No minimum-duty step, stall timer, kick, or integral near zero. */
    common=(hold_common_boost_counts*base+(base+4u)/2u)/(base+4u);
    left_compare=base+common;right_compare+=common;
    limited=left_compare>limit*4u || right_compare>limit*4u;
    if(left_compare>limit*4u)left_compare=limit*4u;
    if(right_compare>limit*4u)right_compare=limit*4u;
    /* Preserve pair sum and common direction. At least half the base remains
       on each wheel; saturation reduces differential authority first. */
    if (command==6u) sync=-sync;
    half=(int32_t)(base/2u);
    low=-(int32_t)left_compare;
    if ((int32_t)right_compare-(int32_t)(limit*4u)>low)
        low=(int32_t)right_compare-(int32_t)(limit*4u);
    if (low < -half) low=-half;
    high=(int32_t)(limit*4u-left_compare);
    if ((int32_t)right_compare < high) high=(int32_t)right_compare;
    if (high > half) high=half;
    if (sync < low) sync=low;
    if (sync > high) sync=high;
    applied_sync=command==6u ? -sync : sync;
    left_compare=(uint32_t)((int32_t)left_compare+sync);
    right_compare=(uint32_t)((int32_t)right_compare-sync);
    starting=observed_stby==0u || observed_direction_pins!=pins;
    if (!starting && observed_pwm_a==left_compare && observed_pwm_b==right_compare) {
        hold_actuator_limited=limited;hold_sync_applied_counts=applied_sync;
        hold_sync_saturation_error_counts=requested_sync-applied_sync;return;
    }
    if (starting) board_motor_off();
#ifndef LEARN_SIMULATOR
    if (starting) {
        GPIOB->BSRR=pins;
        TIM3->CNT=0u; TIM3->CCR4=left_compare; TIM3->CCR3=right_compare;
        TIM3->EGR=TIM_EGR_UG;
        TIM3->CCER=TIM_CCER_CC4E | TIM_CCER_CC3E;
        GPIOA->BSRR=MOTOR_STBY_PIN;
    } else {
        /* Preload both channels together at the next natural PWM boundary.
           Same-direction changes do not insert an off pulse or reset phase. */
        TIM3->CR1 |= TIM_CR1_UDIS;
        TIM3->CCR4=left_compare; TIM3->CCR3=right_compare;
        TIM3->CR1 &= ~TIM_CR1_UDIS;
    }
#endif
    hold_sync_applied_counts=applied_sync;
    hold_sync_saturation_error_counts=requested_sync-applied_sync;
    observed_direction_pins=pins; observed_pwm_a=left_compare;
    observed_pwm_b=right_compare; observed_stby=1u;hold_actuator_limited=limited;
}
void board_hold_start(uint32_t command, uint32_t duty_percent)
{
    if (duty_percent>100u) { board_motor_off(); return; }
    board_hold_start_counts(command,4u*duty_percent,0);
}
#endif
void board_led_set(uint32_t on)
{
    observed_led_on=on!=0u;
#ifndef LEARN_SIMULATOR
    if(on)GPIOC->BRR=1u<<13;else GPIOC->BSRR=1u<<13;
#else
    (void)on;
#endif
}

void board_led_toggle(void)
{
    board_led_set(!observed_led_on);
}

void board_watchdog_start(void)
{
    if (watchdog_started == 0u) {
#ifndef LEARN_SIMULATOR
        /* Independent clock: an accidental debugger halt cannot rely on
           SysTick to stop. IWDG resets to standby if main stops refreshing.
           Approx. 100 ms at LSI=40 kHz; actual LSI varies, not a precise timer. */
        DBGMCU->CR &= ~DBGMCU_CR_DBG_IWDG_STOP;
        RCC->CSR |= RCC_CSR_LSION;
        while ((RCC->CSR & RCC_CSR_LSIRDY) == 0u) { }
        IWDG->KR = 0x5555u;
        IWDG->PR = 3u;              /* Divide LSI clock by 32. */
        IWDG->RLR = 124u;
        while (IWDG->SR != 0u) { }
        IWDG->KR = 0xAAAAu;
        IWDG->KR = 0xCCCCu;
#endif
        /* Start only on first accepted motor command, so initial code
           learning with breakpoints does not cause watchdog resets. */
        watchdog_started = 1u;
    }
    board_watchdog_feed();
}

void board_watchdog_feed(void)
{
#ifndef LEARN_SIMULATOR
    /* Feed even after an IWDG reset, when the hardware IWDG remains enabled. */
    IWDG->KR = 0xAAAAu;
    if (watchdog_started != 0u) {
        DBGMCU->CR &= ~DBGMCU_CR_DBG_IWDG_STOP;
    }
#endif
}

#ifndef ENCODER_H
#define ENCODER_H

#include "board.h"

/* E1=Motor_A socket/TIM4; E2=Motor_B socket/TIM2, not left/right. */
extern volatile int32_t encoder_e1_count;
extern volatile int32_t encoder_e2_count;
extern volatile int32_t encoder_e1_step;
extern volatile int32_t encoder_e2_step;
extern volatile uint32_t encoder_sample_count;

#ifdef LEARN_SIMULATOR
extern volatile uint32_t encoder_sim_e1_raw;
extern volatile uint32_t encoder_sim_e2_raw;
#endif

void encoder_init(void);
void encoder_sample(void);

/* Pure arithmetic helpers, also exercised by the simulator self-tests. */
int32_t encoder_delta16(uint32_t current, uint32_t previous);
int32_t encoder_signed32(uint32_t bits);

#endif

#ifndef PARALLEL_TRIAL_CONFIG_H
#define PARALLEL_TRIAL_CONFIG_H

/* Park baseline scaled by 5.5 PWM percentage points per degree. */
#define PAR_TRIAL_SPEED_KP       27.5f  /* PWM pp / (1000 counts/s) */
#define PAR_TRIAL_SPEED_KI        5.5f  /* PWM pp / 1000 counts */
#define PAR_TRIAL_SPEED_LIMIT    13.75f /* PWM percentage points */
#define PAR_TRIAL_SPEED_MAX_LIMIT 100.0f /* Adjustable PWM limit; default stays 13.75. */

#endif

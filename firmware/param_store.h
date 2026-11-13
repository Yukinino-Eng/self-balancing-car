#ifndef PARAM_STORE_H
#define PARAM_STORE_H
#include <stdint.h>
#ifdef PD_HOLD_TEST
#define PARAM_WORDS 16u
#define PARAM_PAGE_A 0x0800F800u
#define PARAM_PAGE_B 0x0800FC00u
enum { STORE_IDLE=0, STORE_SAVED=1, STORE_BUSY=2, STORE_INVALID=3, STORE_IO_ERROR=4 };
extern volatile uint32_t param_store_command,param_store_status,param_store_valid,param_store_sequence;
extern volatile uint32_t param_store_busy,param_store_request_id,param_store_result_id;
extern volatile uint32_t param_store_dirty;
void param_store_init(void);
void param_store_poll(void);
void param_store_mark_dirty(void);
#ifdef LEARN_SIMULATOR
extern uint32_t param_store_sim_pages[2][256];
extern int32_t param_store_sim_fail_after;
void param_store_sim_clear(void);
#endif
#endif
#endif

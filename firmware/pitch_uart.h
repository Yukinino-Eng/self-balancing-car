#ifndef PITCH_UART_H
#define PITCH_UART_H
#include <stdint.h>
/* USART2 TX=PA2, 115200 8N1, DMA1 channel 7. No receive/control channel. */
enum { PITCH_UART_OFF=0, PITCH_UART_READY=1, PITCH_UART_FAILED=2 };
extern volatile uint32_t pitch_uart_state,pitch_uart_busy;
extern volatile uint32_t pitch_uart_started,pitch_uart_completed;
extern volatile uint32_t pitch_uart_errors;
extern char pitch_uart_buffer[17];
uint32_t pitch_uart_format(char out[17],float pitch,uint32_t valid);
void pitch_uart_init(void);
void pitch_uart_poll(void);
#ifdef LEARN_SIMULATOR
/* 0=pending, 1=DMA complete, 2=DMA error. No peripheral accesses. */
extern uint32_t pitch_uart_sim_result;
#endif
#endif

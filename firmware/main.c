#include "board.h"
#include "runtime.h"
#include "encoder.h"
#include "imu.h"
#include "balance_preview.h"
#include "motor_guard.h"
#include "pd_window.h"
#include "local_control.h"
#include "param_store.h"
#include "peripherals.h"
#include "pitch_uart.h"
#ifdef LEARN_SIMULATOR
#include "selftest.h"
#endif

int main(void)
{
    uint32_t last_feed_ms=0u;
    board_init();
    encoder_init();
    motor_guard_init();
#ifdef LEARN_SIMULATOR
    motor_selftest_run();
#endif
    board_tick_init();
    imu_init();
    balance_preview_init();
    pd_window_init();
    param_store_init();
    local_control_init();
    peripherals_init();
    pitch_uart_init();
    for(;;){
        imu_poll();
        balance_preview_poll();
        motor_guard_poll();
        pd_window_poll();
        local_control_poll();
        param_store_poll();
        if(uptime_ms!=last_feed_ms){
            last_feed_ms=uptime_ms;
            board_watchdog_feed();
        }
        pitch_uart_poll();
        peripherals_poll();
    }
}
void SysTick_Handler(void)
{
    ++uptime_ms;
    local_control_tick();
    encoder_sample();
    pd_window_tick();
}

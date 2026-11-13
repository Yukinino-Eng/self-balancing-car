#ifndef CAR_PERIPHERALS_H
#define CAR_PERIPHERALS_H
#include <stdint.h>
#define PERIPH_TRANSFER_MAX_US 2000u
#define PERIPH_OLED_CHUNK 4u
#define TOF_BUDGET_MS 50u
#define TOF_PERIOD_MS 100u
#define TOF_STALE_MS 300u
#define OLED_REFRESH_MS 200u
#define OBSTACLE_SHOW_MM 2000u
enum { PERIPH_FAIL_NONE=0,PERIPH_FAIL_CLOCK_LOW=1,PERIPH_FAIL_DATA_LOW=2,
       PERIPH_FAIL_ADDRESS_NACK=3,PERIPH_FAIL_DATA_NACK=4,PERIPH_FAIL_DEADLINE=5 };
enum { TOF_WAIT=0,TOF_CONFIG=1,TOF_CALIBRATING=2,TOF_RUNNING=3,TOF_ERROR=4,TOF_WRONG_ID=5 };
enum { OLED_WAIT=0,OLED_INIT=1,OLED_READY=2,OLED_ERROR=3 };
extern volatile uint32_t tof_state,tof_sensor_id,tof_range_mm,tof_range_status,tof_valid;
extern volatile uint32_t tof_sample_count,tof_last_ms,tof_error,tof_error_count;
extern volatile uint32_t oled_state,oled_address,oled_on,oled_error_count;
extern volatile uint32_t peripheral_io_count,peripheral_io_max_us,obstacle_indicator_on;
extern volatile uint32_t tof_mux_readback,tof_start_readback,tof_stream_count,tof_ready_raw;
extern volatile uint32_t peripheral_timeout_count,peripheral_fail_address,peripheral_fail_reg;
extern volatile uint32_t tof_reset_count,tof_boot_readback,peripheral_io_retry_count;
extern volatile uint32_t peripheral_bus_clear_count,peripheral_bus_clear_fail_count,peripheral_fail_kind;
extern volatile uint32_t tof_bus_clear_count,tof_bus_clear_fail_count,tof_transfer_error_count,tof_timeout_count,tof_fail_kind,tof_io_max_us;
extern volatile uint32_t oled_bus_clear_count,oled_bus_clear_fail_count,oled_transfer_error_count,oled_timeout_count,oled_fail_kind,oled_io_max_us;
extern volatile uint32_t oled_dirty_chunks,oled_render_count;
void peripherals_init(void);
void peripherals_poll(void);
#ifdef PERIPHERAL_SELFTEST
void peripherals_selftest_run(void);
#endif
#endif

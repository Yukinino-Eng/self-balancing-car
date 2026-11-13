#ifndef WHEEL_ODOMETRY_H
#define WHEEL_ODOMETRY_H
#include <stdint.h>
/* Working calibration: 48 mm wheel, approximately 1061 counts/revolution. */
#define WHEEL_COUNTS_PER_CM 70.36f
typedef struct { uint32_t left_raw,right_raw; } wheel_origin_t;
typedef struct { float left_cm,right_cm,mean_cm,difference_cm; } wheel_distance_t;
/* Forward is positive for both wheels. Raw E1 decreases when driving forward.
   Unsigned subtraction handles raw counter wrap; each segment must stay below
   2^31 counts on either wheel. Call with a coherent encoder pair. */
void wheel_odometry_origin(wheel_origin_t *origin,uint32_t left,uint32_t right);
void wheel_odometry_measure(const wheel_origin_t *origin,uint32_t left,uint32_t right,
                            wheel_distance_t *distance);
#endif

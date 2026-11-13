#include "wheel_odometry.h"
#include "encoder.h"
void wheel_odometry_origin(wheel_origin_t *origin,uint32_t left,uint32_t right)
{
    origin->left_raw=left;origin->right_raw=right;
}
void wheel_odometry_measure(const wheel_origin_t *origin,uint32_t left,uint32_t right,
                            wheel_distance_t *distance)
{
    distance->left_cm=-(float)encoder_signed32(left-origin->left_raw)/WHEEL_COUNTS_PER_CM;
    distance->right_cm=(float)encoder_signed32(right-origin->right_raw)/WHEEL_COUNTS_PER_CM;
    distance->mean_cm=0.5f*(distance->left_cm+distance->right_cm);
    distance->difference_cm=distance->right_cm-distance->left_cm;
}

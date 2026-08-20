#ifndef EKF_WRAPPER_H
#define EKF_WRAPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void nav_ekf_init(void);
void nav_ekf_set_dummy_imu(uint64_t time_us);
void nav_ekf_update(float* roll, float* pitch, float* yaw);

#ifdef __cplusplus
}
#endif

#endif
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

void app_main(void);

/* Add shared declarations for the main module here. */
// typedef struct {
//     imu_calibrated_t imu;
//     AltData_t alt;
// } SensorDataPacket_t;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */

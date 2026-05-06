#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "sensor_mgr.h"
#include "attitude_ekf.h"

void app_main(void);

/* Add shared declarations for the main module here. */

// Produce a single, coherent view of the rocket’s filtered sensor data
typedef struct {
    uint32_t currTick_ms;            // ms
    LSM6DSV80X_Axes_t currAcc;            // g (calibrated, body-frame)
    LSM6DSV80X_Axes_t currGyro;            // dps (calibrated, body-frame)
    IIS2MDC_Axes_t currMag;              // raw axes (no unit attached)
    float currPress;                 // hPa
    float currTempF;                 // F
    attitude_t attitude;             // Yaw, Pitch, Roll, Tilt in degrees
    float gAltitude;                 // ft (AGL)
    float gVerticalVelocity;         // ft/s (+up)
    float gTotalAcc;                 // g (gravity magnitude)
    float gAccelVelocity;            // unused — reserved
} FusedPacket_t, *FusedPacket_ptr;

//extern FusedPacket_t sensorData;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */

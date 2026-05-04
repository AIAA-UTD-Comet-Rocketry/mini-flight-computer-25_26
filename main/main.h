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
    Accel_Axes_t currAcc;       // mg
    Gyro_Axes_t currGyro;      // mdps
    Mag_Axes_t currMag;
    float currPress;
    float currTempF;
    attitude_t attitude;             // Yaw, Pitch, Roll in degrees
    float gAltitude;
    float gVerticalVelocity;
    float gTotalAcc;                // gravity vector
    float gAccelVelocity;           // m/s^2, gravity removed
} FusedPacket_t, *FusedPacket_ptr;

//extern FusedPacket_t sensorData;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */

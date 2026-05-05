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
    float currPress;           // hPa
    float currTempF;            // F
    attitude_t attitude;             // Yaw, Pitch, Roll in degrees
    float gAltitude;                // ft
    float gVerticalVelocity;        // ft/s
    float gTotalAcc;                // gravity magnititude
    float gAccelVelocity;           // m/s^2, gravity removed
} FusedPacket_t, *FusedPacket_ptr;

//extern FusedPacket_t sensorData;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */

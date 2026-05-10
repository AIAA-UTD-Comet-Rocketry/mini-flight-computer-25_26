#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "sensor_mgr.h"
#include "Fusion.h"

void app_main(void);

/* Add shared declarations for the main module here. */

// Produce a single, coherent view of the rocket’s filtered sensor data
typedef struct {
    uint32_t currTick_ms;            // ms
    FusionVector currAcc;            // g (calibrated, body-frame)
    FusionVector currGyro;            // dps (calibrated, body-frame)
    FusionVector currMag;              // raw axes (no unit attached)
    float currPress;                 // hPa
    float currTempF;                 // F
    FusionEuler orientation;             // Yaw, Pitch, Roll, Tilt in degrees
    FusionVector linearAcc;
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

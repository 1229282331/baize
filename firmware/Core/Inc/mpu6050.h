#ifndef BAIZE_MPU6050_H
#define BAIZE_MPU6050_H

#include "baize_protocol.h"

/* Service once per millisecond. Returns true only for a complete new sample. */
bool mpu6050_poll(uint32_t now_ms, BaizeImu *sample);

#endif

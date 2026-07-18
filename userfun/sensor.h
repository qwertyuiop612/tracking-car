#ifndef SENSOR_H
#define SENSOR_H
#include <stdint.h>

extern int sensor[7];
extern uint8_t idx;

float sensor_detect(void);
float Difspeed();

#endif // SENSOR_H
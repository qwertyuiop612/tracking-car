#ifndef SENSOR_H
#define SENSOR_H
#include <stdint.h>

extern float raw_trace; // 原始传感器读数（未经丢线保持），供 LLM 调参上报

float sensor_detect(void);
float Difspeed();

#endif // SENSOR_H
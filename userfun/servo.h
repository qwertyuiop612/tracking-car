#ifndef SERVO_H
#define SERVO_H

#include "ti_msp_dl_config.h"

void SERVO_Init();
//------0~180------//
void SERVO_set(int deg);

#endif
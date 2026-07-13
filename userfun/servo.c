#include "servo.h"

//-------------------------------------舵机初始化---------------------------------//
void SERVO_init()
{
    DL_Timer_startCounter(SERVO_INST);
    DL_Timer_setCaptureCompareValue(SERVO_INST,50,GPIO_SERVO_C1_IDX);
}

//--------------------------------------角度设置----------------------------------//
void SERVO_set(int deg)
{
    int value = deg * 10 / 9 + 50;
    DL_Timer_setCaptureCompareValue(SERVO_INST, value, GPIO_SERVO_C1_IDX);
}
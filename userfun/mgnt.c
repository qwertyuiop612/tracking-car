#include "mgnt.h"

//---------------------------------电磁铁初始化----------------------------//
void MGNT_Init()
{
    DL_Timer_setCaptureCompareValue(MGNT_INST, 0, GPIO_MGNT_C0_IDX);
}

//----------------------------------磁力设置-------------------------------//
void MGNT_power(int pwm)
{
    DL_Timer_setCaptureCompareValue(MGNT_INST, pwm, GPIO_MGNT_C0_IDX);
}
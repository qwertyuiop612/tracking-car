#include "adc.h"

float ADC_get()
{
    DL_ADC12_startConversion(ADC12_0_INST);
    uint16_t adc_result = DL_ADC12_getMemResult(ADC12_0_INST, ADC12_0_ADCMEM_0);
    float adc_value = adc_result * ADC12_0_ADCMEM_0_REF_VOLTAGE_V / 4096.0;
    return adc_value;
}
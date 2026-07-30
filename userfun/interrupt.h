#ifndef INTERRUPT_H
#define  INTERRUPT_H

#include "ti_msp_dl_config.h"

uint8_t get_key_state(uint32_t key);
void GROUP1_IRQHandler(void);

#endif
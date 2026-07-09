#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

void motor_init(void);
void motor_LT(int speed);
void motor_RT(int speed);
void motor_ST(int speed);
void motor_BK(int speed);
void motor_STOP(void);

#endif // MOTOR_H
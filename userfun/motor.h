#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

//PWM范围为1000到-1000，对应到duty的4000到-4000，负号表示相反

void motor_init(void);
void motor_PWM(int leftPWM,int rightPWM);

#endif // MOTOR_H
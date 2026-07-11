#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

#define PI 3.14
#define ENCODE 260 //编码器线数*减速比
#define WHEEL_DIAMETER 65 //车轮直径/mm

//PWM范围为1000到-1000，对应到duty的4000到-4000，负号表示相反

void motor_init(void);
void motor_PWM(int leftPWM,int rightPWM);

#endif // MOTOR_H
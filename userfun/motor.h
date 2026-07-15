#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

#define PI 3.14
#define ENCODE 390        // 编码器线数*减速比
#define WHEEL_DIAMETER 65 //车轮直径/mm

struct WHEEL {
    float speed;
    float target_speed;
    float last_error;
    float current_error;
};

extern struct WHEEL WHEEL;
extern struct WHEEL RIGHT;
extern struct WHEEL LEFT;
extern uint16_t PWM_1_duty;
extern uint16_t PWM_2_duty;

//PWM范围为1000到-1000，对应到duty的4000到-4000，负号表示相反

void MOTOR_Init(void);
void MOTOR_Enable(void);
void motor_PWM(int leftPWM,int rightPWM);
void direction(uint8_t motor_id , uint8_t dir);

#endif // MOTOR_H
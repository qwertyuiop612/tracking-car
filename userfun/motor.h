#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

#define PI 3.14
#define ENCODE 364        // 编码器线数*减速比
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
extern volatile uint16_t PWM_1_duty;
extern volatile uint16_t PWM_2_duty;
extern volatile uint32_t pid_isr_count;
extern volatile uint32_t pid_run_count;
extern volatile float dbg_error1, dbg_last1;
extern volatile int16_t dbg_pid_out1;
extern volatile int32_t dbg_new_duty1;
extern volatile uint16_t dbg_pwm1_after;
extern volatile float dbg_error2, dbg_last2;
extern volatile int16_t dbg_pid_out2;
extern volatile int32_t dbg_new_duty2;

//PWM范围为1000到-1000，对应到duty的4000到-4000，负号表示相反

void MOTOR_Init(void);
void MOTOR_Enable(void);
void motor_PWM(int leftPWM,int rightPWM);
void direction(uint8_t motor_id , uint8_t dir);

#endif // MOTOR_H
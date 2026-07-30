#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>

#define PI 3.14
#define ENCODE 390        // 编码器线数*减速比
#define WHEEL_DIAMETER 65 //车轮直径/mm

// 速度 PID 参数（可被上位机修改）
extern float v_kp;
extern float v_ki;
extern float v_kd;

struct WHEEL {
    float speed;
    float target_speed;
    float last_error;
    float current_error;
    float err_sum; // PID 积分
};

extern struct WHEEL WHEEL;
extern struct WHEEL RIGHT;
extern struct WHEEL LEFT;
extern volatile uint16_t PWM_1_duty;
extern volatile uint16_t PWM_2_duty;
extern volatile uint8_t motor_brake_flag;

void MOTOR_Init(void);
void motor_PWM(int leftPWM, int rightPWM);
void MOTOR_PID(uint8_t motor_id);
float cal_speed(uint8_t motor_id);

#endif // MOTOR_H
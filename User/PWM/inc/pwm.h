
#ifndef PWM_H
#define PWM_H

#include "stdint.h"
#include "PID.h"
#include "communication.h"
#include "tim.h"
#include "HWTAPP.h"


#define PWM_min 1000
#define PWM_max 2000
#define PWM_mid 1500
#define Step 10        // 每次缓加/缓回的量
#define Tick_ms 50     // 每隔多少 ms 缓加一次(用 HAL_GetTick 计时)

/* 垂直中位 = 水平停位 = 1500(同款电调/电机，不转时的 PWM 一致) */
#define VERTI_MID  1500
#define VERTI_DIVE 1900   // 下潜推力目标(>中位=下沉；若实测相反改成 1100 附近)



#define ANGLE_DEADBAND (5.0f * DEG2RAD)   // 角度死区：±5°以内不纠正角度




extern PID_Regulator_t PitchInPID, PitchOutPID, RollInPID, RollOutPID, YawInPID, YawOutPID;



extern uint8_t flag;          // 垂直：1=上浮(默认)/0=下潜
extern uint8_t moveflag;      // 水平动作：0~6(见 HoriAction_t)
extern uint8_t volatile flag_angle;    // 1=启用水平偏航闭环修正
extern uint8_t volatile PID_ready;     // 中断算完PID后置1；pid_correct纠正完自动置0

extern volatile float Target_roll;     // 期望横滚(弧度)，防侧翻目标一般设 0
extern volatile float Target_pitch;    // 期望俯仰(弧度)
extern volatile float Target_yaw;      // 期望偏航(弧度)

extern uint16_t vertPWM_base; // 垂直公共基线(上浮~下潜缓动)
extern uint16_t vertPWM[4];   // [0]=桨0 [1]=桨1 [2]=桨2 [3]=桨3
extern uint16_t horiPWM[4];   // [0]=左前 [1]=左后 [2]=右前 [3]=右后


void pid_init(void);                  
void Pwm_init(void);    

void pid_calculate_all(void);

void vertical_action(void);               // 上浮/下潜 基线缓动
void horizontal_action(void);             // moveflag -> 水平基值
void action_all(void);

void vert_pid_correct(void);              // 垂直防侧翻 -> TIM1
void hor_pid_correct(void);               // 水平基值+偏航 -> TIM8
void pid_correct(void);                 

void YAW_set(void);
#endif

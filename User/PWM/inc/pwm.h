
#ifndef PWM_H
#define PWM_H

#include "stdint.h"
#include "PID.h"
#include "communication.h"
#include "tim.h"
#include "HWTAPP.h"

/* ---------- 动作枚举 ---------- */
typedef enum {
    HORI_STOP = 0,
    HORI_FRONT,          // 前进
    HORI_BACK,           // 后退
    HORI_LEFT,           // 左移
    HORI_RIGHT,          // 右移
    HORI_CLOCKWISE,      // 顺时针/右转
    HORI_ANTICLOCKWISE   // 逆时针/左转
} HoriAction_t;

/* 垂直两态：上浮=回中位靠浮力浮起；下潜=缓加下沉推力到底 */
typedef enum {
    VERTI_DIVE   = 0,     // 下潜
    VERTI_SURFACE = 1     // 上浮(默认)
} VertState_t;

/* ---------- 全局接口(由你在控制循环里赋值) ---------- */
extern uint8_t flag;          // 垂直：1=上浮(默认)/0=下潜
extern uint8_t moveflag;      // 水平动作：0~6(见 HoriAction_t)
extern uint8_t flag_angle;    // 1=启用水平偏航闭环修正
extern uint8_t PID_ready;     // 中断算完PID后置1；pid_correct纠正完自动置0

extern float Target_roll;     // 期望横滚(弧度)，防侧翻目标一般设 0
extern float Target_pitch;    // 期望俯仰(弧度)
extern float Target_yaw;      // 期望偏航(弧度)

extern uint16_t vertPWM_base; // 垂直公共基线(上浮~下潜缓动)
extern uint16_t vertPWM[4];   // [0]=桨0 [1]=桨1 [2]=桨2 [3]=桨3
extern uint16_t horiPWM[4];   // [0]=左前 [1]=左后 [2]=右前 [3]=右后

/* ---------- 函数(调用约定见 pwm.c 顶部) ---------- */
void pid_init(void);                      // 上电调一次，配置各 PID 参数

/* 定时器中断里调用：算 PID(只算不写电机)，算完在主循环触发 pid_correct */
void vert_pid_calculate(void);            // 垂直 roll/pitch 双环
void hor_pid_calculate(void);             // 水平偏航双环

/* 主循环里调用：做动作(设基值/缓动) */
void vertical_action(void);               // 上浮/下潜 基线缓动
void horizontal_action(void);             // moveflag -> 水平基值

/* 主循环里调用：PID 纠正(检测 PID_ready==1，写电机)。顺序须 vert 在前、hor 在后 */
void vert_pid_correct(void);              // 垂直防侧翻 -> TIM1
void hor_pid_correct(void);               // 水平基值+偏航 -> TIM8

#endif

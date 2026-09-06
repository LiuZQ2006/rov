#ifndef RM_FRAME_C_PID_H
#define RM_FRAME_C_PID_H

#include <stddef.h>

typedef struct {
    float ref;
    float fdb;
    float err[4];
    float errSum;
    float kp;
    float ki;
    float kd;
    float componentKp;
    float componentKi;
    float componentKd;
    float componentKpMax;
    float componentKiMax;
    float componentKdMax;
    float output;
    float outputMax;
}PID_Regulator_t;

#define INRANGE(NUM, MIN, MAX) \
{\
    if(NUM<MIN){\
        NUM=MIN;\
    }else if(NUM>MAX){\
        NUM=MAX;\
    }\
}

void PID_Init(PID_Regulator_t *p,float kp, float ki, float kd, float pM, float iM, float dM, float oM);//对应最开始的初始化
void PID_Reset(PID_Regulator_t *PIDInfo);//对应reset
void PID_Reload(PID_Regulator_t *PIDInfo,PID_Regulator_t *pidRegulator);//对应第二个reset，带参那个
float PIDCalc(PID_Regulator_t *PIDInfo, float target, float feedback);//对应计算pid输出
float PIDCalcMAX(PID_Regulator_t *PIDInfo, float target, float feedback, float max);//对应计算pid输出，带最大输出限制
float normalize(float a);
float PIDCalcYaw(PID_Regulator_t *PIDInfo, float target, float feedback);

#endif

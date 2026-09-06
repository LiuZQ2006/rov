#include "PID.h"
void PID_Init(PID_Regulator_t *p,float kp, float ki, float kd, float pM, float iM, float dM, float oM)
{
    *p = (PID_Regulator_t){0}; 
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->componentKpMax = pM;
    p->componentKiMax = iM;
    p->componentKdMax = dM;
    p->outputMax = oM;
}
void PID_Reset(PID_Regulator_t *PIDInfo) {
    PIDInfo->ref = 0;
    PIDInfo->fdb = 0;
    PIDInfo->err[0] = 0;
    PIDInfo->err[1] = 0;
    PIDInfo->err[2] = 0;
    PIDInfo->err[3] = 0;
    PIDInfo->errSum = 0;

    PIDInfo->componentKp = 0;
    PIDInfo->componentKi = 0;
    PIDInfo->componentKd = 0;

    PIDInfo->output = 0;

}
void PID_Reload(PID_Regulator_t *PIDInfo,PID_Regulator_t *pidRegulator) {
    if(pidRegulator != NULL)*PIDInfo = *pidRegulator;
}

/* 角度归一到 [-PI, PI]，PID 用 */
float normalize(float a)
{
    while (a >  3.14159265f) a -= 6.28318530f;
    while (a < -3.14159265f) a += 6.28318530f;
    return a;
}

/**
 * Compute the PID control output.
 * @param target   /Reference (setpoint) value.
 * @param feedback /Feedback (measured) value.
 * @return /Control output.
 */
/**
 * 计算pid算法的控制量
 * @param target 目标量
 * @param feedback 反馈量
 * @return 控制量
 */
float PIDCalc(PID_Regulator_t *PIDInfo, float target, float feedback) {
    PIDInfo->fdb = feedback;
    PIDInfo->ref = target;
    PIDInfo->err[3] = PIDInfo->ref - PIDInfo->fdb;
    PIDInfo->componentKp = PIDInfo->err[3] * PIDInfo->kp;

    if (PIDInfo->ki != 0.0f) {
    PIDInfo->errSum += PIDInfo->err[3];
    INRANGE(PIDInfo->errSum, -PIDInfo->componentKiMax / PIDInfo->ki,
            PIDInfo->componentKiMax / PIDInfo->ki);
    } 
    else {
    PIDInfo->errSum = 0.0f;   /* 纯P: 不累积积分 */
    }
    
    PIDInfo->componentKi = PIDInfo->errSum * PIDInfo->ki;
    PIDInfo->componentKd = (PIDInfo->err[3] - PIDInfo->err[2]) * PIDInfo->kd;

    INRANGE(PIDInfo->componentKp, -1 * PIDInfo->componentKpMax, PIDInfo->componentKpMax);
    INRANGE(PIDInfo->componentKi, -1 * PIDInfo->componentKiMax, PIDInfo->componentKiMax);
    INRANGE(PIDInfo->componentKd, -1 * PIDInfo->componentKdMax, PIDInfo->componentKdMax);

    PIDInfo->output = PIDInfo->componentKp + PIDInfo->componentKi + PIDInfo->componentKd;
    INRANGE(PIDInfo->output, -1 * PIDInfo->outputMax, PIDInfo->outputMax);

    PIDInfo->err[2] = PIDInfo->err[3];
    return PIDInfo->output;
}

float PIDCalcYaw(PID_Regulator_t *PIDInfo, float target, float feedback) {
    PIDInfo->fdb = feedback;
    PIDInfo->ref = target;
    PIDInfo->err[3] = PIDInfo->ref - PIDInfo->fdb;
    PIDInfo->err[3] = normalize(PIDInfo->err[3]);
    PIDInfo->componentKp = PIDInfo->err[3] * PIDInfo->kp;
    if (PIDInfo->ki != 0.0f) {
    PIDInfo->errSum += PIDInfo->err[3];
    INRANGE(PIDInfo->errSum, -PIDInfo->componentKiMax / PIDInfo->ki,
            PIDInfo->componentKiMax / PIDInfo->ki);
    } 
    else {
    PIDInfo->errSum = 0.0f;   /* 纯P: 不累积积分 */
    }
    
    PIDInfo->componentKi = PIDInfo->errSum * PIDInfo->ki;
    PIDInfo->componentKd = (PIDInfo->err[3] - PIDInfo->err[2]) * PIDInfo->kd;

    INRANGE(PIDInfo->componentKp, -1 * PIDInfo->componentKpMax, PIDInfo->componentKpMax);
    INRANGE(PIDInfo->componentKi, -1 * PIDInfo->componentKiMax, PIDInfo->componentKiMax);
    INRANGE(PIDInfo->componentKd, -1 * PIDInfo->componentKdMax, PIDInfo->componentKdMax);

    PIDInfo->output = PIDInfo->componentKp + PIDInfo->componentKi + PIDInfo->componentKd;
    INRANGE(PIDInfo->output, -1 * PIDInfo->outputMax, PIDInfo->outputMax);

    PIDInfo->err[2] = PIDInfo->err[3];
    return PIDInfo->output;
}



float PIDCalcMAX(PID_Regulator_t *PIDInfo, float target, float feedback, float max) {
    PIDInfo->outputMax = max;
    return PIDCalc(PIDInfo, target, feedback);
}


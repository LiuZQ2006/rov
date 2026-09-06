#include "pwm.h"

static TIM_HandleTypeDef* const VERT_TIM[4] = { &htim1, &htim1, &htim1, &htim1 };
static const uint32_t VERT_CH[4] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };
static TIM_HandleTypeDef* const HORI_TIM[4] = { &htim8, &htim8, &htim8, &htim8 };
static const uint32_t HORI_CH[4] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };

/* ============ 状态(由串口指令/主循环设) ============ */
uint8_t volatile flag_angle = 0;      // 1=启用水平偏航闭环
uint8_t volatile PID_ready = 0;       // 中断里 calculate 完成后置1，pid_correct 纠正完置0

volatile float Target_roll  = 0.0f;   // 期望横滚(0=不侧翻)
volatile float Target_pitch = 0.0f;   // 期望俯仰
volatile float Target_yaw   = 0.0f;   // 期望偏航(弧度)

uint16_t vertPWM[4]   = { VERTI_MID, VERTI_MID, VERTI_MID, VERTI_MID };
uint16_t vertPWM_base = VERTI_MID;   // 垂直公共基线(上浮=中位 / 下潜缓加到 DIVE)
uint16_t horiPWM[4]   = { PWM_mid, PWM_mid, PWM_mid, PWM_mid };

/* ============ 水平动作表(中心=1500，相对中位 ±80/±40) ============ */
static const uint16_t HORI_ACTION[][4] = {
/* STOP           */ {1500, 1500, 1500, 1500},
/* FRONT 前进     */ {1420, 1580, 1580, 1420}, // LF正桨进,LR反桨进,RF反桨进,RR正桨进
/* BACK  后退     */ {1580, 1420, 1420, 1580}, // 全部反向推力
/* LEFT  左横移   */ {1580, 1580, 1420, 1420}, // 向左平移
/* RIGHT 右横移   */ {1420, 1420, 1580, 1580}, // 向右平移
/* CLOCKWISE 顺时针原地转  */ {1420, 1420, 1420, 1420},
/* ANTICLOCKWISE 逆时针原地转 */ {1580, 1580, 1580, 1580}, 
};

#define HORI_ACTION_NUM (sizeof(HORI_ACTION) / sizeof(HORI_ACTION[0]))

/* 正反桨/转向符号(方向反了就整组取反) */
static const int8_t VERTI_SIGN[4] = { -1, 1, 1, -1 };
static const int8_t HORI_SIGN[4]  = { 1, -1, -1, 1 };
static const int32_t VERTI_TRIM[4] = { 0, 0, 0, 0 };   // 每桨悬浮微调，歪了再加

/* 水平动作基值缓存 + 变更检测 */
static uint16_t hori_base[4];
static uint8_t  last_move = 0xFF;

PID_Regulator_t PitchInPID, PitchOutPID, RollInPID, RollOutPID, YawInPID, YawOutPID;

/* 中断里 calculate 算出的原始修正量(存到这，主循环 pid_correct 用) */
static volatile float v_roll, v_pitch;   /* 垂直 roll/pitch 内环输出 */
static volatile float h_yaw;             /* 水平偏航修正强度 */


void pid_init(void)
{
    PID_Init(&PitchInPID, 8, 0, 0, 100, 50, 50, 200);
    PID_Init(&PitchOutPID, 1, 0.005, 1, 5, 2.5, 2.5, 10);
    PID_Init(&RollInPID, 8, 0, 0, 100, 50, 50, 200);
    PID_Init(&RollOutPID, 0.5, 0.002, 1, 5, 2.5, 2.5, 10);
    PID_Init(&YawInPID, 5, 0, 0, 200, 100, 100, 400);
    PID_Init(&YawOutPID, 2, 0.01, 2, 10, 5, 5, 20);
}

/* ---------- 垂直 PID 计算(中断)：roll/pitch 双环 ---------- */
static void vert_pid_calculate(void)
{
    float trv = PIDCalc(&RollOutPID,  Target_roll,  fAngle[0]);  /* 外环: 角度差->目标角速度 */
    float tpv = PIDCalc(&PitchOutPID, Target_pitch, fAngle[1]);
    v_roll  = PIDCalc(&RollInPID,  trv, -fGyro[0]);              /* 内环: 角速度差->推力修正 */
    v_pitch = PIDCalc(&PitchInPID, tpv,  fGyro[1]);
}

/* ---------- 水平 PID 计算(中断)：偏航双环 ---------- */
static void hor_pid_calculate(void)
{
    if (flag_angle) {

        float tyv = PIDCalcYaw(&YawOutPID, Target_yaw, fAngle[2]);
        h_yaw = PIDCalc(&YawInPID, tyv, fGyro[2]);               /* 内环 */
    } else {
        h_yaw = 0.0f;
    }
}

void pid_calculate_all(void)
{
    vert_pid_calculate();
    hor_pid_calculate();
    PID_ready = 1;
}

/* ---------- 垂直 做动作(主循环)：上浮/下潜基线缓动 ---------- */
void vertical_action(void)
{
    static uint32_t s_last = 0;
    int32_t target = (flag == 0) ? VERTI_DIVE : VERTI_MID;  /* 下潜加到 DIVE / 上浮回中位 */

    if ((uint32_t)(HAL_GetTick() - s_last) < Tick_ms) return;   // 每 Tick_ms 缓一步
    s_last = HAL_GetTick();

    if (vertPWM_base < target) {
        vertPWM_base += Step;
        if (vertPWM_base > target) vertPWM_base = (uint16_t)target;
    } else if (vertPWM_base > target) {
        vertPWM_base -= Step;
        if (vertPWM_base < target) vertPWM_base = (uint16_t)target;
    }
}

/* ---------- 水平 做动作(主循环)：moveflag -> 基值(变更才更新) ---------- */
void horizontal_action(void)
{
    if (moveflag == last_move) return;
    if (moveflag >= HORI_ACTION_NUM) return;   // 越界保护
    last_move = moveflag;
    int move_idx;
    switch (moveflag) {
        case HORI_STOP:          move_idx = 0;
        break;
        case HORI_FRONT:         move_idx = 1;
        break;
        case HORI_BACK:          move_idx = 2;
        break;
        case HORI_LEFT:          move_idx = 3;
        break;
        case HORI_RIGHT:         move_idx = 4;
        break;
        case HORI_CLOCKWISE:     move_idx = 5;
        break;
        case HORI_ANTICLOCKWISE: move_idx = 6;
        break;
    }
    for (int i = 0; i < 4; i++)
        hori_base[i] = HORI_ACTION[move_idx][i];
}

/* ---------- 垂直 PID 纠正(主循环)：差分防侧翻 -> TIM1 ---------- */
void vert_pid_correct(void)
{
    static const int8_t factors[4][2] = {
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}   /* roll, pitch 差动 */
    };
    for (int i = 0; i < 4; i++) {
        int32_t sign = VERTI_SIGN[i];
        int32_t pwm  = (int32_t)vertPWM_base + VERTI_TRIM[i]
                     - sign * (int32_t)(v_roll * factors[i][0] + v_pitch * factors[i][1]);
        if (pwm < PWM_min) pwm = PWM_min;
        if (pwm > PWM_max) pwm = PWM_max;
        vertPWM[i] = (uint16_t)pwm;
        __HAL_TIM_SET_COMPARE(VERT_TIM[i], VERT_CH[i], vertPWM[i]);
    }
}

void hor_pid_correct(void)
{
    for (int i = 0; i < 4; i++) {
        int8_t  sign = HORI_SIGN[i];
        int32_t base = hori_base[i];
        int32_t pwm  = base + ((i < 2) ? sign : -sign) * (int32_t)h_yaw;
        if (pwm < PWM_min) pwm = PWM_min;
        if (pwm > PWM_max) pwm = PWM_max;
        horiPWM[i] = (uint16_t)pwm;
        __HAL_TIM_SET_COMPARE(HORI_TIM[i], HORI_CH[i], horiPWM[i]);
    }
}

void pid_correct(void)
{
        if (!PID_ready) return;
    vert_pid_correct();
    hor_pid_correct();
        PID_ready = 0;   // 纠正完成
}
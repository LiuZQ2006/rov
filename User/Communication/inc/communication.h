#ifndef COMMUNICATION_H
#define COMMUNICATION_H


#include "usart.h"
#include "PID.h"

extern volatile uint8_t code_ready;
extern uint8_t code[64];
extern uint8_t flag;           
extern uint8_t moveflag;  
extern PID_Regulator_t PitchInPID, PitchOutPID, RollInPID, RollOutPID, YawInPID, YawOutPID;

#define DOWN 0xAA
#define UP 0xEE

#define HORI_STOP           0x00
#define HORI_FRONT          0x01
#define HORI_BACK           0x02
#define HORI_LEFT           0x04
#define HORI_RIGHT          0x08
#define HORI_CLOCKWISE      0x10
#define HORI_ANTICLOCKWISE  0x20

//包头
#define CODE_BEGIN_ONE          0x27
#define CODE_BEGIN_TWO          0xCC

//包尾
#define CODE_END    0xAF 

//使能
#define ENABLE_CMD  0x00
#define ENABLE_ON   0xFF
#define ENABLE_OFF  0x00
#define ENABLE_LENGTH   1

//看门狗
#define WATCH_DOG_CMD   0xD9

//平移
#define STEER_CMD   0x01
#define STEER_LENGTH 4
#define STEER_MAX   32768

//转向
#define TURN_CMD    0x02
#define TURN_LENGTH 2
#define TURN_MAX    32768

//浮沉
#define SINK_CMD    0x03
#define SINK_LENGTH 2
#define SINK_MAX    32768

//灯光
#define LIGHT_CMD   0x11
#define LIGHT_LENGTH    4

//机械手
#define MECHARM_CMD 0x57
#define MECHARM_LENGTH  4

//配置
#define SET_CMD 0x57
#define SET_LENGTH 9

//看门狗上限设置
#define SET_WATCHDOG_LIMIT  0x00

//PitchInPID, PitchOutPID, RollInPID, RollOutPID, YawInPID, YawOutPID;

//设置PitchInPID
#define SET_PitchInPID_KP   0x01
#define SET_PitchInPID_KI   0x02
#define SET_PitchInPID_KD   0x03

//设置PitchOutPID
#define SET_PitchOutPID_KP  0x11
#define SET_PitchOutPID_KI  0x12
#define SET_PitchOutPID_KD  0x13

//RollInPID
#define SET_RollInPID_KP  0x21
#define SET_RollInPID_KI  0x22
#define SET_RollInPID_KD  0x23

//RollOutPID
#define SET_RollOutPID_KP  0x31
#define SET_RollOutPID_KI  0x32
#define SET_RollOutPID_KD  0x33

//YawInPID
#define SET_YawInPID_KP  0x41
#define SET_YawInPID_KI  0x42
#define SET_YawInPID_KD  0x43

//YawOutPID
#define SET_YawOutPID_KP  0x51
#define SET_YawOutPID_KI  0x52
#define SET_YawOutPID_KD  0x53

//设置PID_ERROR
#define PID_ERROR_PITCH 0x61
#define PID_ERROR_ROLL  0x62
#define PID_ERROR_YAW   0x63

//STOP
#define SET_PID_DEAD_ZONE        0x71

//轮询 
#define ASK_CMD         0XAA
#define ASK_ENABLE      0x00
#define ASK_WATCHDOG_CURRENT    0x01
#define ASK_WATCHDOG_UPPER_LIMIT    0x02
#define ASK_STEER_INTENSITY_X   0x10
#define ASK_STEER_INTENSITY_Y   0x11
#define ASK_ROTATE_INTENSITY    0x12
#define ASK_SINK_INTENSITY      0x13
#define ASK_LIGHT_INTENSITY_LEFT    0x20
#define ASK_LIGHT_INTENSITY_RIGHT   0x21
#define ASK_MECHARM_SERVO_1     0x30
#define ASK_MECHARM_SERVO_2     0x31




void Feed_communication(uint8_t *data, uint8_t len);
void code_analyze(void);
#endif


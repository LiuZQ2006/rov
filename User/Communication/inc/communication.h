#ifndef COMMUNICATION_H
#define COMMUNICATION_H


#include "usart.h"

extern uint8_t mode_flag;
extern uint8_t code[64];
extern uint8_t flag;           
extern uint8_t moveflag;  

#define DOWN 0x01
#define UP 0x02

#define HORI_STOP          0
#define HORI_FRONT         1
#define HORI_BACK          2
#define HORI_LEFT          3
#define HORI_RIGHT         4
#define HORI_CLOCKWISE     5
#define HORI_ANTICLOCKWISE 6



#endif
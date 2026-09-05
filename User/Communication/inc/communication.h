#ifndef COMMUNICATION_H
#define COMMUNICATION_H


#include "usart.h"

extern uint8_t mode_flag;
extern uint8_t code[64];

#define STOP 0x00
#define DOWN 0x01
#define UP 0x02
#define FORWARD 0x04
#define BACKWARD 0x08
#define LEFT 0x10
#define RIGHT 0x20




#endif
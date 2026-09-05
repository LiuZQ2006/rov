#ifndef _HWTAPP_H_
#define _HWTAPP_H_

#include "wit_c_sdk.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "main.h"
#include "tim.h"


#define DEG2RAD  0.01745329252f   


extern volatile float fAcc[3], fGyro[3], fAngle[3],fTemp;
extern volatile uint8_t data_ready_flag;

void HWT_data(void);
void HWT_Tick(void);
void HWT_Init(void);


#endif


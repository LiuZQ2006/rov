#ifndef _HWTAPP_H_
#define _HWTAPP_H_

#include "wit_c_sdk.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "main.h"
#include "tim.h"

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define TEMP_UPDATE     0x10
#define READ_UPDATE		0x80

#define DEG2RAD  0.01745329252f   


extern float fAcc[3], fGyro[3], fAngle[3],fTemp;

void HWT_data(void);
void HWT_Tick(void);
void HWT_Init(void);


#endif


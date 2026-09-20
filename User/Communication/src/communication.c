#include "communication.h"
#include "stdio.h"
uint8_t flag = 1;            
uint8_t moveflag = HORI_STOP;  
volatile uint8_t code_ready=0;
uint8_t code[64] = {0};


//看门狗
static uint16_t WATCHDOG_CURRENT;
static uint16_t WATCHDOG_UPPER_LIMIT;

//平移 
static int16_t STEER_INTENSITY_X;
static int16_t STEER_INTENSITY_Y;

//转向
static int16_t ROTATE_INTENSITY;

//浮沉
static int16_t SINK_INTENSITY;

//灯光
static uint16_t LIGHT_INTENSITY_LEFT,LIGHT_INTENSITY_RIGHT; //作为灯光的CCR

//机械手
static uint16_t  MECHARM_SERVO_1,MECHARM_SERVO_2 ;









void Feed_communication(uint8_t *data, uint8_t len)
{
    for (int i = 0; i < len; i++)
    {
        code[i] = data[i];
    }
    code_ready = 1;
}

void code_analyze(void)
{
    if(!code_ready) return;
    switch(code[0])

    code_ready = 0;  
}






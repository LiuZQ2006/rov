#include "communication.h"

uint8_t flag = 1;            
uint8_t moveflag = HORI_STOP;  
uint8_t data_ready=0;
uint8_t code[64] = {0};


void Feed_com(uint8_t *data, uint8_t len)
{
    for (int i = 0; i < len; i++)
    {
        code[i] = data[i];
    }
    data_ready = 1;
}





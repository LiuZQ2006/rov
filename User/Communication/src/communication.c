#include "usart.h"

uint8_t mode_flag = 0;
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





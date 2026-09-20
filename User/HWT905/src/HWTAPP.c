#include "HWTAPP.h"



//这里是一些全局变量的定义啥的
volatile float fAcc[3], fGyro[3], fAngle[3],fTemp;
static int i,iBuff;
static uint8_t g_imu_rx_buf[128]; 
volatile uint8_t data_ready_flag = 0;

static volatile int s_query_inflight = 0;   /* 1=已发查询、在等应答 */
static volatile int s_new_data = 0;         /* 1=收到一帧新数据    */
static uint32_t s_query_tick = 0;           /* 查询发出时刻(超时用) */



static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	HAL_UART_Transmit(&huart2, p_data, uiSize, 50);
}

static void Delayms(uint16_t ucMs)
{
	HAL_Delay(ucMs);
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	 (void)uiReg; (void)uiRegNum;   // 这两个参数用不到
    s_query_inflight = 0;          // 应答收完，允许发下一帧
    s_new_data = 1;                // 有新数据了

}

void HWT_Init(){
    HAL_TIM_Base_Start_IT(&htim14);
	WitInit(WIT_PROTOCOL_MODBUS, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(CopeSensorData);
	HAL_UARTEx_ReceiveToIdle_IT(&huart2, g_imu_rx_buf, 128);
	WitDelayMsRegister(Delayms);
}



void HWT_Tick(void)
    {
        /* 收完才发下一帧 */
        if (!s_query_inflight)
        {
            WitReadReg(AX, 12);             /* 发查询 */
            s_query_inflight = 1;
            s_query_tick = HAL_GetTick();
        }
        else if (HAL_GetTick() - s_query_tick > 10)
        {
            s_query_inflight = 0;         
        }
    }

void HWT_data(void){
        if (s_new_data)
        {
            s_new_data = 0;
            for(i = 0; i < 3; i++)
            {
                fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f*DEG2RAD;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f*DEG2RAD;
            }
            data_ready_flag = 1;
//            printf("acc:%.3f %.3f %.3f gyro:%.3f %.3f %.3f angle:%.3f %.3f %.3f temp:%.1f\r\n",
//                fAcc[0],fAcc[1],fAcc[2], fGyro[0],fGyro[1],fGyro[2],
//                fAngle[0],fAngle[1],fAngle[2], fTemp);
        }
}

void Target_reset(void){
    while(!data_ready_flag);
    Target_roll = fAngle[0];
    Target_pitch = fAngle[1];
    Target_yaw = fAngle[2];
    data_ready_flag = 0;
}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
		if(huart->Instance==USART2)
	{
		for(uint8_t j=0;j<Size;j++)
		{
			WitSerialDataIn(g_imu_rx_buf[j]);
		}
		memset(g_imu_rx_buf,0,sizeof(g_imu_rx_buf));
	HAL_UARTEx_ReceiveToIdle_IT(&huart2,g_imu_rx_buf,sizeof(g_imu_rx_buf));
	}
}



int fputc(int a,FILE*f)
{
	HAL_UART_Transmit(&huart1,(uint8_t *)&a,1,HAL_MAX_DELAY);
	return a;
}




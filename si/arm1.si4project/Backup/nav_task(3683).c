#ifndef  __GOL_NAV_TASK_C__
#define  __GOL_NAV_TASK_C__
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"

#include "string.h"
#include "UartAdapter.h"
#include "drv_usart.h"
#include "nav_task.h"
#include "frame_analysis.h"
#include "gnss.h"

typedef struct
{
    IMU_PARSE_DATA_TypeDef 	imuInfo;
    GPSDataTypeDef			gnssInfo;
} CombineDataTypeDef;
CombineDataTypeDef combineData;

//422
//Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, imu_comm5_len, fpga_comm5_rxbuffer);
//232 uart4
//uint32_t gd32_usart_write(uint8_t *buffer, uint32_t size)

void nav_task(void* arg)
{

    while( 1 )
    {
        if(pdTRUE == xSemaphoreTake( xnvaTaskSemaphore, portMAX_DELAY))
        {
            memcpy((void*)&combineData.imuInfo.accelGrp[0], (void*)&imuParseData.accelGrp[0], sizeof(IMU_PARSE_DATA_TypeDef));
            memcpy((void*)&combineData.gnssInfo.timestamp, (void*)&hGPSData.timestamp, sizeof(GPSDataTypeDef));
            Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, sizeof(CombineDataTypeDef), (uint8_t*)&combineData.imuInfo.accelGrp[0]);
        }
    }
}

#endif


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
#include "drv_rtc.h"
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
    rtc_update_struct *rtc;
    uint8_t status;

    while( 1 )
    {
        if(pdTRUE == xQueueReceive( xNavQueue, &status, portMAX_DELAY))
        {
            //if(pdTRUE == xSemaphoreTake( xnvaTaskSemaphore, portMAX_DELAY))
            if(1 == status)//IMU数据
            {
                rtc = rtc_update();
                combineData.imuInfo.gps_second = rtc->gpsTime;
                memcpy((void*)&combineData.imuInfo.accelGrp[0], (void*)&imuParseData.accelGrp[0], sizeof(IMU_PARSE_DATA_TypeDef));
                //Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, sizeof(IMU_PARSE_DATA_TypeDef), (uint8_t*)&combineData.imuInfo.gps_second);
            }
            else if(2 == status)//GNSS数据
            {
                memcpy((void*)&combineData.gnssInfo.timestamp, (void*)&hGPSData.timestamp, sizeof(GPSDataTypeDef));
                //Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, sizeof(GPSDataTypeDef), (uint8_t*)&combineData.gnssInfo.timestamp);
            }
        }
    }
}

#endif


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



/*****************kalman algorithm headers**************************/
#include "constant.h"
#include "matrix.h"
#include "inavgnss.h"
#include "inavins.h"
#include "inavlog.h"

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
#if 0
void UpdateCompensate(void)
{
    COMPENSATE_PARAMS *pCompensate = NULL;

    pCompensate	=	GetCompensateParmsPointer();

    if(NULL == pCompensate)
    {
        inav_log(LOG_ERR, "pCompensate is NULL, UpdateCompensate fail");
    }

    //zhang
}

void match_imu_data(IMUDATA *data, MIX_DATA_TypeDef* mixd)
{
    data->accm[0] = mixd->accelGrp[0];
    data->accm[1] = mixd->accelGrp[1];
    data->accm[2] = mixd->accelGrp[2];

    data->gyro[0] = mixd->gyroGrp[0];
    data->gyro[1] = mixd->gyroGrp[1];
    data->gyro[2] = mixd->gyroGrp[2];

    data->roll 	  = mixd->roll;
    data->pitch   = mixd->pitch;
    data->heading = mixd->azimuth;

    data->second  = mixd->gps_sec;
}

void match_gnss_data(I_NAV_GNSS_RESULT  * data, ARM1_TO_KALAM_MIX_TypeDef* mixd)
{
    data->gpssecond = mixd->GPS_Sec;
    data->heading = mixd->Heading;
    data->pitch = mixd->pitch;
    data->roll = mixd->roll;
    data->latitude 	= mixd->lat;
    data->longitude = mixd->lon;
    data->altitude 	= mixd->alt;
    data->vn = mixd->vN;
    data->ve = mixd->vE;
    data->vu = mixd->vZ;
    data->gnssstatus = mixd->rtk_status;
    data->latstd = mixd->sigma_lat;
    data->logstd = mixd->sigma_lon;
    data->hstd = mixd->sigma_alt;
    data->vnstd = mixd->sigma_vn;
    data->vestd = mixd->sigma_ve;
    data->vustd = mixd->sigma_vz;

}

void UpdateGnssAndInsData(void)
{
    IMUDATA *pImuData = NULL;
    I_NAV_GNSS_RESULT *pGnssResult = NULL;
    MIX_DATA_TypeDef* pMix = frame_get_ptr();
    ARM1_TO_KALAM_MIX_TypeDef*  pGnss = gnss_get_algorithm_dataPtr();
    pImuData = GetNavIncImuPointer();

    if(NULL != pImuData)
    {
        //save last time data
        pImuData->gyro_pre[0] 	=	pImuData->gyro[0];
        pImuData->gyro_pre[1] 	=	pImuData->gyro[1];
        pImuData->gyro_pre[2] 	=	pImuData->gyro[2];
        pImuData->accm_pre[0] 	=	pImuData->accm[0];
        pImuData->accm_pre[1] 	=	pImuData->accm[1];
        pImuData->accm_pre[2] 	=	pImuData->accm[2];
        //zhang write fpga data to pImuData
//        inav_log(LOG_DEBUG, "update imu data from fpga data");
        match_imu_data(pImuData, pMix);
    }
    else
    {
        inav_log(LOG_ERR, "pImuData is NULL");
    }

    pGnssResult = GetGnssResultPointer();

    if(NULL != pGnssResult)
    {
        //zhang  write fpga data to pGnssResult
//        inav_log(LOG_DEBUG, "update gnss result from fpga data");
        match_gnss_data(pGnssResult, pGnss);
    }
    else
    {
        inav_log(LOG_ERR, "pGnssResult is NULL");
    }
}
#endif

void nav_task(void* arg)
{
    //rtc_update_struct *rtc;
    uint8_t status;

    while( 1 )
    {
        if(pdTRUE == xQueueReceive( xNavQueue, &status, portMAX_DELAY))
        {
            //if(pdTRUE == xSemaphoreTake( xnvaTaskSemaphore, portMAX_DELAY))
            if(1 == status)//IMU数据
            {
                //rtc = rtc_update();
                //combineData.imuInfo.gps_second = rtc->gpsTime;
                memcpy((void*)&combineData.imuInfo.counter, (void*)&imuParseData.counter, sizeof(IMU_PARSE_DATA_TypeDef));
                Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, sizeof(IMU_PARSE_DATA_TypeDef), (uint8_t*)&combineData.imuInfo.counter);
                //gd32_usart_write((uint8_t*)&combineData.imuInfo.gps_second, sizeof(IMU_PARSE_DATA_TypeDef));
            }
            else if(2 == status)//GNSS数据
            {
                memcpy((void*)&combineData.gnssInfo.timestamp, (void*)&hGPSData.timestamp, sizeof(GPSDataTypeDef));
                Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, sizeof(GPSDataTypeDef), (uint8_t*)&combineData.gnssInfo.timestamp);
                //gd32_usart_write((uint8_t*)&combineData.gnssInfo.timestamp, sizeof(GPSDataTypeDef));
            }
        }
    }
}

#endif


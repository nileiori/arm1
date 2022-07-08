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
//#include "frame_analysis.h"
//#include "gnss.h"
#include "drv_rtc.h"



/*****************kalman algorithm headers**************************/
#include "constant.h"
#include "matrix.h"
#include "inavgnss.h"
#include "inavins.h"
#include "inavlog.h"

QueueHandle_t xNavQueue;
uint8_t xNavStatus;

CombineDataTypeDef combineData;

//422
//Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, imu_comm5_len, fpga_comm5_rxbuffer);
//232 uart4
//uint32_t gd32_usart_write(uint8_t *buffer, uint32_t size)

void UpdateCompensate(void)
{
    COMPENSATE_PARAMS *pCompensate = NULL;

    pCompensate	=	GetCompensateParmsPointer();

    if(NULL == pCompensate)
    {
        inav_log(INAVMD(LOG_ERR), "pCompensate is NULL, UpdateCompensate fail");
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

void nav_init(void)
{

}

int nav_process(void)
{
    uint16_t len;
    double mGnssArmLength[3] = {0};
    I_NAV_INS *pINavIns = NULL ;
    I_NAV_GNSS_RESULT *pGnssResult = NULL;
    //I_NAV_CAN_RESULT *pCanResult = NULL;
    STATIC_DETECTION *pStaticDetect = NULL;
    COMPENSATE_PARAMS *pCompensate = NULL;
    unsigned int inavloglevel;
    unsigned int mPrintCnt = 0;
    double* mMatrixTest = NULL;
//TEST
    rtc_update_struct* rtc_update_info;
    uint32_t pregpsTime = 0;
    uint32_t pregpsWeek = 0;
    uint32_t curgpsTime = 0;
    uint32_t curgpsWeek = 0;
    uint32_t dt = 0;

    inav_set_loglevel(LOG_WARNING);
    InitialNavIncParm();
    InitialGnssResultParm();
    InitialStaticDetectParm();
    //InitialCanResultParm();

    UpdateCompensate();


    pINavIns 	= 	GetNavIncPointer();
    pGnssResult 	= 	GetGnssResultPointer();
    //pCanResult	=     GetCanResultPointer();
    pStaticDetect	=	GetStaticDetectPointer();
    pCompensate	=	GetCompensateParmsPointer();

    if(		NULL == pINavIns
            || 	NULL == pGnssResult
            //||   NULL == pCanResult
            || 	NULL == pStaticDetect
            || 	NULL == pCompensate )
    {
        inav_log(INAVMD(LOG_ERR), "some infomation is NULL");
    }

    inav_set_loglevel(LOG_DEBUG);
    inav_log(INAVMD(LOG_DEBUG), "LOG_DEBUG TEST");
    //PrintOutGNSSMsg();
    //PrintOutInsMsg();
    //mMatrixTest = eyes(15);
    //PrintOutMatrixMsg(15,15 ,mMatrixTest, "mMatrixTest");
    //free(mMatrixTest);
    rtc_update_info = rtc_update();
    pregpsTime = rtc_update_info->gpsTime;
    pregpsWeek = rtc_update_info->gpsWeek;

    unsigned int mgpslostpos = 10;
    unsigned int mgpslostnum = 10;
    unsigned int mindex = 0;

    while(1)
    {
        rtc_update_info = rtc_update();
        curgpsTime = rtc_update_info->gpsTime;
        curgpsWeek = rtc_update_info->gpsWeek;

        if(curgpsWeek == pregpsWeek)
        {
            dt = curgpsTime - pregpsTime;
        }
        else
        {

            dt = curgpsTime - pregpsTime + 604800;
        }

        if(dt >= 5)
        {
            pregpsTime = curgpsTime;
            pregpsWeek = curgpsWeek;
            mindex++;

            if(mindex > mgpslostpos && mindex <= (mgpslostpos + mgpslostnum))
            {
                if(mindex == (mgpslostpos + 1) )
                {
                    inav_log(INAVMD(LOG_DEBUG), "gps start lost");
                }

                pGnssResult->gnssstatus = NAV_GNSS_STATUS_LOST;
            }
            else if(mindex > (mgpslostpos + mgpslostnum))
            {
                if(mindex == (mgpslostpos + mgpslostnum + 1) )
                {
                    inav_log(INAVMD(LOG_DEBUG), "gps end lost");
                }

                mindex = mgpslostpos + mgpslostnum + 1;
            }

            UpdateCompensate();
            InterfaceKalman();
        }


    }


}
extern I_NAV_INS g_NAV_INS;
extern I_NAV_GNSS_RESULT g_NAV_GNSS_RESULT;
void nav_task(void* arg)
{
    uint8_t status;
    InitialNavIncParm();
    //初始化静态检查参数
    InitialStaticDetectParm();

    while( 1 )
    {
        if(pdTRUE == xQueueReceive( xNavQueue, &status, portMAX_DELAY))
        {
            //if(pdTRUE == xSemaphoreTake( xnvaTaskSemaphore, portMAX_DELAY))
            if(1 == status)//IMU数据
            {
                memcpy((void*)&combineData.imuInfo.counter, (void*)&imuParseData.counter, sizeof(IMU_PARSE_DATA_TypeDef));
                Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, sizeof(IMU_PARSE_DATA_TypeDef), (uint8_t*)&combineData.imuInfo.counter);
                //gd32_usart_write((uint8_t*)&combineData.imuInfo.gps_second, sizeof(IMU_PARSE_DATA_TypeDef));
            }
            else if(2 == status)//GNSS数据
            {
                memcpy((void*)&combineData.gnssInfo.timestamp, (void*)&hGPSData.timestamp, sizeof(GPSDataTypeDef));
                Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, sizeof(GPSDataTypeDef), (uint8_t*)&combineData.gnssInfo.timestamp);
                //gd32_usart_write((uint8_t*)&combineData.gnssInfo.timestamp, sizeof(GPSDataTypeDef));
                GetNavIncData(&g_NAV_INS.imu, &combineData);
                GetNavGnssData(&g_NAV_GNSS_RESULT, &combineData);
                UpdateCompensate();
                InterfaceKalman();
            }

        }
    }
}

#endif


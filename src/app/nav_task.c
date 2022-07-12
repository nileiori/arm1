#ifndef  __GOL_NAV_TASK_C__
#define  __GOL_NAV_TASK_C__

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


//422
//Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, imu_comm5_len, fpga_comm5_rxbuffer);
//232 uart4
//uint32_t gd32_usart_write(uint8_t *buffer, uint32_t size)

uint8_t xImuStatus;
uint8_t xGnssStatus;
uint8_t xCommStatus;


CombineDataTypeDef combineData;

extern I_NAV_INS g_NAV_INS;
extern I_NAV_GNSS_RESULT g_NAV_GNSS_RESULT;
extern COMPENSATE_PARAMS g_Compensate_Params;

EXPORT_RESULT  g_Export_Result;

EXPORT_RESULT* SaveResult(EXPORT_RESULT* result, I_NAV_INS* nav_ins, I_NAV_GNSS_RESULT* gnssResult)
{
    result->imusecond = nav_ins->imu.second;
    //-----陀螺-----
	result->gyro[0] = nav_ins->imu.gyro[0];
	result->gyro[1] = nav_ins->imu.gyro[1];
	result->gyro[2] = nav_ins->imu.gyro[2];
	//---加速度计---
	result->accm[0] = nav_ins->imu.accm[0];
	result->accm[1] = nav_ins->imu.accm[1];
	result->accm[2] = nav_ins->imu.accm[2];
	
    result->dt = nav_ins->dt;
    result->att[0] = nav_ins->ins.att[0] * RAD2DEG; //pitch
    result->att[1] = nav_ins->ins.att[1] * RAD2DEG; //roll
    result->att[2] = nav_ins->ins.att[2] * RAD2DEG; //heading

    result->v_n[0] = nav_ins->ins.vn[0];
    result->v_n[1] = nav_ins->ins.vn[1];
    result->v_n[2] = nav_ins->ins.vn[2];

    result->pos[0] = nav_ins->ins.pos[0] * RAD2DEG;
    result->pos[1] = nav_ins->ins.pos[1] * RAD2DEG;
    result->pos[2] = nav_ins->ins.pos[2];

    result->insvnstd[0] = nav_ins->ins.vnstd[0];
    result->insvnstd[1] = nav_ins->ins.vnstd[1];
    result->insvnstd[2] = nav_ins->ins.vnstd[2];

    result->insposstd[0] = nav_ins->ins.posstd[0] * RAD2DEG;
    result->insposstd[1] = nav_ins->ins.posstd[1] * RAD2DEG;
    result->insposstd[2] = nav_ins->ins.posstd[2];

	result->nsv = gnssResult->nsv;
	result->gnsslocatestatus = gnssResult->gnsslocatestatus;
    result->gnssstatus = gnssResult->gnssstatus;
    result->gpsweek = gnssResult->gpsweek;
    result->gpssecond = gnssResult->gpssecond;

    result->pitch = gnssResult->pitch;
    result->roll = gnssResult->roll;
    result->heading = gnssResult->heading;

    result->latitude = gnssResult->latitude;
    result->longitude = gnssResult->longitude;
    result->altitude = gnssResult->altitude;
	
    result->ve = gnssResult->ve;
    result->vn = gnssResult->vn;
    result->vu = gnssResult->vu;

    result->latstd = gnssResult->latstd;
    result->logstd = gnssResult->logstd;
    result->hstd = gnssResult->hstd;

	result->hdgstddev = gnssResult->hdgstddev;
    result->ptchstddev = gnssResult->ptchstddev;
    
    result->vestd = gnssResult->vestd;
    result->vnstd = gnssResult->vnstd;
    result->vustd = gnssResult->vustd;

    return result;
}

unsigned int GetCompensateParm(COMPENSATE_PARAMS *p, void *comm)
{
    p->gnssArmLength[0] = 0.0;
    p->gnssArmLength[1] = 0.0;
    p->gnssArmLength[2] = 0.0;
    p->heading = 0.0;
    return 0;
}

void Kalman_smooth(void)
{
    GetNavIncData(&g_NAV_INS.imu, &combineData);
    GetNavGnssData(&g_NAV_GNSS_RESULT, &combineData);
    GetCompensateParm(&g_Compensate_Params, NULL);
    InterfaceKalman();
    SaveResult(&g_Export_Result, &g_NAV_INS, &g_NAV_GNSS_RESULT);
}

void InitialCompensateParm()
{
    memset(&g_Compensate_Params, 0, sizeof(COMPENSATE_PARAMS));
    g_Compensate_Params.gnssArmLength[0] = -0.4188;
    g_Compensate_Params.gnssArmLength[1] = -0.1746;
    g_Compensate_Params.gnssArmLength[2] = -0.099;
}
extern void Oscilloscope(void);
void nav_task(void)
{
	static uint8_t nav_init = 0;

	if(nav_init == 0)
	{
		nav_init = 1;
	    InitialNavIncParm();
	    //初始化静态检查参数
	    InitialStaticDetectParm();
	    //初始化补偿量
	    InitialCompensateParm();  
	}
    if(xImuStatus)//IMU数据
    {
        xImuStatus = 0;
        memcpy((void*)&combineData.imuInfo.counter, (void*)&imuParseData.counter, sizeof(IMU_PARSE_DATA_TypeDef));
        //Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, sizeof(IMU_PARSE_DATA_TypeDef), (uint8_t*)&combineData.imuInfo.counter);
        //gd32_usart_write((uint8_t*)&combineData.imuInfo.gps_second, sizeof(IMU_PARSE_DATA_TypeDef));
        //卡尔曼滤波
        Kalman_smooth();
    }
    if(xGnssStatus)//GNSS数据
    {
        xGnssStatus = 0;
        memcpy((void*)&combineData.gnssInfo.timestamp, (void*)&hGPSData.timestamp, sizeof(GPSDataTypeDef));
        //Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, sizeof(GPSDataTypeDef), (uint8_t*)&combineData.gnssInfo.timestamp);
        //gd32_usart_write((uint8_t*)&combineData.gnssInfo.timestamp, sizeof(GPSDataTypeDef));
    }
    if(xCommStatus)//串口发送至上位机
    {
        xCommStatus = 0;
        frame_pack_and_send(&g_Export_Result, &hGPSData);
        //frame_writeDram();
        //Oscilloscope();
    }
}

#endif


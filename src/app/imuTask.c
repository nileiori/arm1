#ifndef __GOL_IMUTASK_C__
#define __GOL_IMUTASK_C__
#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "config.h"

#include "drv_rtc.h"

#include "uartadapter.h"
#include "gnss.h"
#include "main.h"
#include "imuTask.h"
#include "nav_task.h"

#include "m_queue.h"




static uint8_t fpga_comm5_rxbuffer[IMU_BUFFER_SIZE];
static uint16_t imu_comm5_len = 0;
uint8_t imu_ready = 0;


uint8_t xImuStatus;


void imu_comm5_rx(void)
{
    imu_comm5_len = Uart_RecvMsg(UART_RXPORT_COMPLEX_12, 23, fpga_comm5_rxbuffer);

    if(imu_comm5_len == 23)
    {
        imu_ready = 1;
    }

}

void imu_comm5_task(void)
{
    if(imu_ready)//pdMS_TO_TICKS(100)
    {
        imu_ready = 0;
        frame_fill_imu(fpga_comm5_rxbuffer, imu_comm5_len); //send to fpga

        xImuStatus = 1;
        imu_comm5_len = 0;
    }

}


#endif


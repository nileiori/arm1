/*!
    \file    main.c
    \brief   led spark with systick

    \version 2016-08-15, V1.0.0, firmware for GD32F4xx
    \version 2018-12-12, V2.0.0, firmware for GD32F4xx
    \version 2020-09-30, V2.1.0, firmware for GD32F4xx
    \version 2022-03-09, V3.0.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "exmc_sram.h"
#include "config.h"
#include "drv_gpio.h"
#include "drv_usart.h"
#include "drv_rtc.h"
#include "drv_timer.h"
#include "public.h"
#include "pin_numbers_def.h"
#include "frame_analysis.h"
#include "uartadapter.h"
#include "gnss.h"
#include "taskschedule.h"
#include "computerFrameParse.h"
#include "fwdog.h"
//#include "fmc_operation.h"
#include "DRamAdapter.h"
#include "nav_task.h"
#include "Time_Unify.h"

#include "m_queue.h"

#define	ARM1_TO_ARM2_IO		GD32F450_PA12_PIN_NUM
#define	SYN_ARM_IO			GD32F450_PE2_PIN_NUM
#define	FPGA_TO_ARM1_INT	GD32F450_PA2_PIN_NUM

#define	ARM1_TO_ARM2_SYN 	gd32_pin_write(SYN_ARM_IO, PIN_LOW)


#define GNSS_BUFFER_SIZE            1024
#define IMU_BUFFER_SIZE             100

#define TASK_GNSS_COMM2_BIT ( 1 << 0 )
#define TASK_GNSS_COMM3_BIT ( 1 << 1 )
#define TASK_IMU_BIT 		( 1 << 2 )
#define ALL_SYNC_BITS 		( TASK_GNSS_COMM2_BIT | TASK_GNSS_COMM3_BIT | TASK_IMU_BIT )

/* SRAM */
uint8_t fpgaBuf[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
static uint8_t fpga_comm2_rxbuffer[GNSS_BUFFER_SIZE];
static uint8_t fpga_comm2_parse_rxbuffer[GNSS_BUFFER_SIZE];
static uint16_t gnss_comm2_len = 0;
static uint16_t gnss_comm2_parse_len = 0;

static uint8_t fpga_comm3_rxbuffer[GNSS_BUFFER_SIZE];
static uint8_t fpga_comm3_parse_rxbuffer[GNSS_BUFFER_SIZE];
static uint16_t gnss_comm3_len = 0;
static uint16_t gnss_comm3_parse_len = 0;

static uint8_t fpga_comm5_rxbuffer[IMU_BUFFER_SIZE];
static uint16_t imu_comm5_len = 0;


//static uint8_t comm2_fifo[GNSS_BUFFER_SIZE*2];
//static uint8_t comm3_fifo[GNSS_BUFFER_SIZE*2];
CirQueue_t comm2_queue;
CirQueue_t comm3_queue;


uint8_t fpga_syn = 0;//fpga同步
uint8_t gnss_comm2_ready = 0;
uint8_t gnss_comm3_ready = 0;
uint8_t imu_ready = 0;


uint8_t fpga_dram_wr[1000] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,};
uint8_t fpga_dram_rd[1000];


void syn_arm2(void)
{
    gd32_pin_write(SYN_ARM_IO, PIN_LOW);
    gd32_pin_write(SYN_ARM_IO, PIN_LOW);
    gd32_pin_write(SYN_ARM_IO, PIN_HIGH);
}

void fpga_dram_fill(void)
{
    uint16_t index = 0;

    for(index = 0; index < 1000; index++)
    {
        fpga_dram_wr[index] = index;
    }
}

void fpga_dram_test(void)
{
    fpga_dram_fill();
    DRam_Write(0, (uint16_t*)fpga_dram_wr, 256);
    DRam_Read(0, (uint16_t*)fpga_dram_rd, 256);
}

static void prvSetupHardware( void )
{
    SystemInit();

    SystemCoreClockUpdate();

    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    systick_config();

#ifdef	configUse_SEGGER_RTT
    SEGGER_RTT_Init();
    SEGGER_RTT_printf(0, "\r\nCK_SYS is %d", rcu_clock_freq_get(CK_SYS));
    SEGGER_RTT_printf(0, "\r\nCK_AHB is %d", rcu_clock_freq_get(CK_AHB));
    SEGGER_RTT_printf(0, "\r\nCK_APB1 is %d", rcu_clock_freq_get(CK_APB1));
    SEGGER_RTT_printf(0, "\r\nCK_APB2 is %d", rcu_clock_freq_get(CK_APB2));
#endif
}
#if 0
/////////////////////////////power down///////////////////////////////////////////
void sleep_int_install(void)
{
//    uint8_t i;
    irq_priority priority =
    {
        .nvic_irq_pre_priority = 0,
        .nvic_irq_sub_priority = 0
    };

    pin_irq_install(  SYN_ARM_IO, PIN_MODE_INPUT_PULLDOWN,
                      PIN_IRQ_MODE_RISING,
                      NULL,
                      NULL,
                      &priority);
}

void sleep_int_uninstall(void)
{

    gd32_pin_irq_enable(SYN_ARM_IO, PIN_IRQ_DISABLE);
}

void system_config_before_sleep(void)
{
    NVIC_DisableIRQ(SysTick_IRQn);
    timer_disable(TIMER0);
    timer_disable(TIMER1);
    timer_disable(TIMER13);
    //adc_disable(ADC0);
}

void system_config_after_wakeup(void)
{

    prvSetupHardware();


}

void system_halt(void)
{
    system_config_before_sleep();
    sleep_int_install();
    pmu_to_deepsleepmode(PMU_LDO_LOWPOWER, PMU_LOWDRIVER_ENABLE, WFI_CMD);
    sleep_int_uninstall();
    system_config_after_wakeup();

}
#endif
void gnss_comm2_rx(void)
{
    gnss_comm2_len = Uart_RecvMsg(UART_RXPORT_COMPLEX_9, GNSS_BUFFER_SIZE, fpga_comm2_rxbuffer);
    EnCirQueue(fpga_comm2_rxbuffer, gnss_comm2_len,comm2_queue);

}

void gnss_comm2_parse(uint8_t* pData, uint16_t dataLen)
{
    uint8_t temp;
    do
    {
        if(ByteDeCirQueue(&temp, comm2_queue))
        {
            if(('$' == temp) || ('#' == temp))
            {
                gnss_comm2_parse_len = 0;
                fpga_comm2_parse_rxbuffer[gnss_comm2_parse_len] = temp;
                gnss_comm2_parse_len++;
            }
            else if(0x0a == temp)
            {
                if(fpga_comm2_parse_rxbuffer[gnss_comm2_parse_len - 1] == 0x0d)//回车换行结尾
                {
                    fpga_comm2_parse_rxbuffer[gnss_comm2_parse_len] = temp;
                    gnss_comm2_parse_len++;

                    if((0 == strncmp("#AGRICA", (char const *)fpga_comm2_parse_rxbuffer, 7))
                            || (0 == strncmp("#BESTPOSA", (char const *)fpga_comm2_parse_rxbuffer, 9))
                            || (0 == strncmp("#BESTVELA", (char const *)fpga_comm2_parse_rxbuffer, 9))
                            || (0 == strncmp("$GNTRA", (char const *)fpga_comm2_parse_rxbuffer, 6))
                            || (0 == strncmp("$GPTRA", (char const *)fpga_comm2_parse_rxbuffer, 6))
                      )
                    {
#ifdef configUse_debug
                        //Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, gnss_comm2_parse_len, fpga_comm2_parse_rxbuffer);
                        //gd32_usart_write(fpga_comm2_parse_rxbuffer, gnss_comm2_parse_len);
#endif
                        gnss_fill_data(fpga_comm2_parse_rxbuffer, gnss_comm2_parse_len);
                        gnss_comm2_parse_len = 0;
                        break;
                    }
                }
            }
            else
            {
                fpga_comm2_parse_rxbuffer[gnss_comm2_parse_len] = temp;
                gnss_comm2_parse_len++;
            }
        }
    }
    while(1);
}

void gnss_comm3_rx(void)
{

    gnss_comm3_len = Uart_RecvMsg(UART_RXPORT_COMPLEX_10, GNSS_BUFFER_SIZE, fpga_comm3_rxbuffer);
    EnCirQueue(fpga_comm3_rxbuffer, gnss_comm3_len,comm3_queue);
}

void gnss_comm3_parse(uint8_t* pData, uint16_t dataLen)
{
    uint8_t temp;
    do
    {
        if(ByteDeCirQueue(&temp, comm3_queue))
        {
            if(('$' == temp) || ('#' == temp))
            {
                gnss_comm3_parse_len = 0;
                fpga_comm3_parse_rxbuffer[gnss_comm3_parse_len] = temp;
                gnss_comm3_parse_len++;
            }
            else if(0x0a == temp)
            {
                if(fpga_comm3_parse_rxbuffer[gnss_comm3_parse_len - 1] == 0x0d)//回车换行结尾
                {
                    fpga_comm3_parse_rxbuffer[gnss_comm3_parse_len] = temp;
                    gnss_comm3_parse_len++;

                    if((0 == strncmp("#HEADINGA", (char const *)fpga_comm3_parse_rxbuffer, 9))
                            || (0 == strncmp("#BESTPOSA", (char const *)fpga_comm3_parse_rxbuffer, 9))
                            || (0 == strncmp("#BESTVELA", (char const *)fpga_comm3_parse_rxbuffer, 9))
                            ||(0 == strncmp("$GNRMC", (char const *)fpga_comm3_parse_rxbuffer, 6))
                            ||(0 == strncmp("$GNGGA", (char const *)fpga_comm3_parse_rxbuffer, 6))
                            || (0 == strncmp("$GNVTG", (char const *)fpga_comm3_parse_rxbuffer, 6))
                            || (0 == strncmp("$GNZDA", (char const *)fpga_comm3_parse_rxbuffer, 6))
                            || (0 == strncmp("$GPZDA", (char const *)fpga_comm3_parse_rxbuffer, 6))
                            || (0 == strncmp("$GNTRA", (char const *)fpga_comm3_parse_rxbuffer, 6))
                            || (0 == strncmp("$GPTRA", (char const *)fpga_comm3_parse_rxbuffer, 6))

                      )
                    {
#ifdef configUse_debug
                        //Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, gnss_comm3_parse_len, fpga_comm3_parse_rxbuffer);
                        //gd32_usart_write(fpga_comm3_parse_rxbuffer, gnss_comm3_parse_len);
#endif
                        gnss_fill_data(fpga_comm3_parse_rxbuffer, gnss_comm3_parse_len);
                        gnss_comm3_parse_len = 0;
                        break;
                    }
                }
            }
            else
            {
                fpga_comm3_parse_rxbuffer[gnss_comm3_parse_len] = temp;
                gnss_comm3_parse_len++;
            }
        }
    }
    while(1);

}

void imu_comm5_rx(void)
{
    imu_comm5_len = Uart_RecvMsg(UART_RXPORT_COMPLEX_12, 23, fpga_comm5_rxbuffer);

    if(imu_comm5_len == 23)
    {
        imu_ready = 1;
    }

}

//////////////////////////////////////////////////////////////////////////////////
void fpga_int_hdr(void *args)
{
    fpga_syn = 1;

    gnss_comm2_rx();

    gnss_comm3_rx();

    imu_comm5_rx();
}

gtime_t tt;
void arm_syn_int_hdr(void *args)
{
    DRam_Read(0, (uint16_t*)&tt.time, 6);
}

void gpio_init(void)
{
    irq_priority priority =
    {
        .nvic_irq_pre_priority = 4,
        .nvic_irq_sub_priority = 0
    };

    //gd32_pin_mode(ARM1_TO_ARM2_IO, PIN_MODE_OUTPUT);
    gd32_pin_mode(SYN_ARM_IO, PIN_MODE_OUTPUT);

    pin_irq_install(  FPGA_TO_ARM1_INT, PIN_MODE_INPUT_PULLDOWN,
                      PIN_IRQ_MODE_RISING,
                      fpga_int_hdr,
                      NULL,
                      &priority);
    pin_irq_install(  ARM1_TO_ARM2_IO, PIN_MODE_INPUT,
                      PIN_IRQ_MODE_RISING_FALLING,
                      arm_syn_int_hdr,
                      NULL,
                      &priority);
}

static void peripherals_init(void)
{
    exmc_sram_init();

    // Uart
    //Uart_TxInit(UART_TXPORT_RS232_1, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_DEFAULT, UART_ENABLE);
    //Uart_RxInit(UART_RXPORT_RS232_1, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_DEFAULT, UART_ENABLE);

    Uart_TxInit(UART_TXPORT_COMPLEX_8, UART_BAUDRATE_230400BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS422, UART_ENABLE);
    Uart_RxInit(UART_RXPORT_COMPLEX_8, UART_BAUDRATE_230400BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS422, UART_ENABLE);

    Uart_TxInit(UART_TXPORT_COMPLEX_9, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);
    Uart_RxInit(UART_RXPORT_COMPLEX_9, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);

    Uart_TxInit(UART_TXPORT_COMPLEX_10, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);
    Uart_RxInit(UART_RXPORT_COMPLEX_10, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);

    Uart_TxInit(UART_TXPORT_COMPLEX_11, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);
    Uart_RxInit(UART_RXPORT_COMPLEX_11, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);

    Uart_TxInit(UART_TXPORT_COMPLEX_12, UART_BAUDRATE_921600BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);
    Uart_RxInit(UART_RXPORT_COMPLEX_12, UART_BAUDRATE_921600BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);

    frame_init();

    gd32_hw_usart_init();

    gpio_init();

    gd32_timer_init();

    rtc_configuration();

//    fwdog_init();
}

void gnss_comm2_task(void)
{

    gnss_comm2_parse(fpga_comm2_rxbuffer, gnss_comm2_len); //send to fpga
    gnss_comm2_len = 0;

}

void gnss_comm3_task(void)
{

    gnss_comm3_parse(fpga_comm3_rxbuffer, gnss_comm3_len); //send to fpga
    gnss_comm3_len = 0;

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

void adjust_rtc(void)
{
    static uint8_t ucTimeOneSecondChangeHour = 0xff;
    static uint8_t rtcAdjInit = 0;

    //开机等待时间有效校准一次，之后每小时校准一次
    if(rtcAdjInit == 0)
    {
        if(INS_EOK == rtc_gnss_adjust_time())
        {
            rtcAdjInit = 1;
            rtc_update();
            ucTimeOneSecondChangeHour = RSOFT_RTC_HOUR;
        }
    }
    else
    {
        if(ucTimeOneSecondChangeHour != RSOFT_RTC_HOUR)
        {
            ucTimeOneSecondChangeHour = RSOFT_RTC_HOUR;

            rtc_gnss_adjust_time();
        }
    }
}
uint16_t rtc_update_flg = 0;
void rtc_syn(void)
{
    if(rtc_update_flg > INS_TICK_PER_SECOND)
    {
        rtc_update_flg = 0;
        rtc_update();
    }
}
//uint32_t val;
//uint32_t mcount=0;
extern void lfs_selftest(void);
int main(void)
{

    prvSetupHardware();
    peripherals_init();
    comm2_queue = CirQueueGenericCreate(GNSS_BUFFER_SIZE*2);
    comm3_queue = CirQueueGenericCreate(GNSS_BUFFER_SIZE*2);
//    fpga_dram_test();
#ifdef configUse_debug
    dbg_periph_enable(DBG_FWDGT_HOLD);
    dbg_periph_enable(DBG_WWDGT_HOLD);
#endif
#ifdef INS_USING_UART4_DMA0
    //Public_SetTestTimer(comm_handle, 10);
#endif
    //lfs_selftest();
    while(1)
    {
//        SysTick->LOAD=0xfffff0;//时间加载(SysTick->LOAD为24bit)
//        SysTick->VAL   = 0;/* Load the SysTick Counter Value */
//        SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
//                         SysTick_CTRL_TICKINT_Msk   |
//                         SysTick_CTRL_ENABLE_Msk;
        gnss_comm2_task();
        gnss_comm3_task();
        imu_comm5_task();
        nav_task();
        comm_handle();
        adjust_rtc();
        rtc_syn();
//        val = 0xfffff0 - SysTick->VAL;
//        mcount++;
//        if(mcount > 500000)
//        {
//            mcount = 0;
//            //Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, 4, (uint8_t*)&val);
//        }
//        SysTick->CTRL  &= ~SysTick_CTRL_ENABLE_Msk;
        //TimerTaskScheduler();
    }
}

EventStatus system_1s_task(void)
{

    rtc_update();
    return ENABLE;
}


/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(UART4, (uint8_t)ch);

    while(RESET == usart_flag_get(UART4, USART_FLAG_TBE));

    return ch;
}


/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"

#include "UartAdapter.h"
#include "drv_usart.h"
#include "nav_task.h"


SemaphoreHandle_t xnvaTaskSemaphore  = NULL;
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
			
        }
    }
}




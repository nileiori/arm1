#ifndef NAV_TASK_H_INCLUDED
#define NAV_TASK_H_INCLUDED

#undef COMMON_EXT
#ifdef  __GOL_NAV_TASK_C__
    #define COMMON_EXT
#else
    #define COMMON_EXT extern
#endif	

#ifdef __cplusplus
extern "C" {
#endif

#include "insdef.h"
#include "gd32f4xx.h"
#include "frame_analysis.h"
#include "gnss.h"

#include "FreeRTOS.h"
#include "semphr.h"

typedef struct
{
    IMU_PARSE_DATA_TypeDef 	imuInfo;
    GPSDataTypeDef			gnssInfo;
} CombineDataTypeDef;

COMMON_EXT SemaphoreHandle_t xnvaTaskSemaphore;
COMMON_EXT QueueHandle_t xNavQueue;
COMMON_EXT uint8_t xNavStatus;


void nav_task(void* arg);


#ifdef __cplusplus
}
#endif

#endif 





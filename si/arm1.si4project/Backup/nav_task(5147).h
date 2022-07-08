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

#include "FreeRTOS.h"
#include "semphr.h"

COMMON_EXT SemaphoreHandle_t xnvaTaskSemaphore;

void nav_task(void* arg);


#ifdef __cplusplus
}
#endif

#endif 





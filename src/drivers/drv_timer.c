
#include "stdio.h"
#include "drv_timer.h"
//#include "nav_task.h"

extern uint8_t xCommStatus;
/**************************************************************************************************/
/*                                                                                                */
/*                    					timer3_init                                               */
/*                                                                                                */
/**************************************************************************************************/
/**
 * @brief   timer3 initialization  1000Hz
 * @param   none
 *
 * @retval  None
 */
void timer3_init(void)
{
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER3);

    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);//200M

    nvic_irq_enable(TIMER3_IRQn, 5, 0);

    timer_deinit(TIMER3);

    /* TIMER3 configuration */
    timer_initpara.prescaler         = 200 - 1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 5000;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;

    timer_init(TIMER3, &timer_initpara);
    /* enable TIMERI auto-reload shadow function */
    timer_auto_reload_shadow_enable( TIMER3 );
    timer_update_source_config(TIMER3, TIMER_UPDATE_SRC_REGULAR);
    /* clear the update interrupt bit */
    timer_interrupt_flag_clear( TIMER3, TIMER_INT_UP );
    /* enable the update interrupt */
    timer_interrupt_enable( TIMER3, TIMER_INT_UP);
    /* TIMERI counter enable */
    timer_enable( TIMER3);
}



void TIMER3_IRQHandler()
{

    if(SET == timer_interrupt_flag_get(TIMER3, TIMER_INT_UP))
    {
        timer_interrupt_flag_clear( TIMER3, TIMER_INT_UP );

        xCommStatus = 1;
    }
}

void gd32_timer_init(void)
{
    
    timer3_init();
}



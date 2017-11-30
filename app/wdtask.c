/*
 * Copyright 2014, 2017, Jernej Kovacic
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software. If you wish to use our Amazon
 * FreeRTOS name, please do so in a fair use way that does not cause confusion.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @file
 *
 * Implementation of functions that interact with the
 * selected watchdog timer, e.g. configuration and reloading.
 *
 * The file also includes a simple ISR that resets the board
 * if the watchdog reached timeout.
 *
 * @author Jernej Kovacic
 */


#include <stdint.h>
#include <stddef.h>

#include <FreeRTOS.h>
#include <task.h>

#include "wdtask.h"
#include "watchdog.h"
#include "bsp.h"
#include "scb.h"


#define WD_UNSPECIFIED       ( (uint8_t) -1 )

/* A constant indicating that no watchdog has been selected yet */
static uint8_t __wdNr = WD_UNSPECIFIED;


/*
 * Selected watchdog's interrupt handler.
 *
 * It resets the board.
 */
static void __wdHandler(void)
{
    scb_reset();

    /*
     * The interrupt flag is intentionally not cleared.
     * If the command above does not succeed for any reason,
     * the board will automaticall reset on next timeout.
     */
}


/**
 * Configures the selected watchdog timer.
 * The watchdog is configured to reset after the first time out.
 * The function also starts the selected watchdog.
 *
 * @note Once this function successfully configures one watchdog,
 *       it cannot be called (and configure other watchdogs)
 *       anymore. In this case, nothing will be done and
 *       pdFAIL will be returned immediately.
 *
 * @param wd - desired watchdog timer (between 0 and 1)
 * @param timeoutMs -watchdog's time out period in milliseconds
 *
 * @return pdPASS on success, pdFAIL if 'wd' is invalid
 */
int16_t watchdogInit( uint8_t wd, uint32_t timeoutMs )
{
    int16_t retVal = pdFAIL;

    /*
     * Immediately calculate the required load for the selected
     * watchdog, using relevant clocks' know frequencies.
     * Note: the frequency must be divided by 1000 first, otherwise
     * the multiplication of 'timeoutMs' and a frequency may
     * exceed the uint32_t range!
     */
    const uint32_t timeoutTicks =
        ( 1==wd ?
          timeoutMs * (configPIOSC_CLOCK_HZ / 1000) :
          timeoutMs * (configCPU_CLOCK_HZ /1000) );

    /* The function only succeeds if no watchdog has been configured yet */
    if ( __wdNr==WD_UNSPECIFIED && wd<BSP_NR_WATCHDOGS )
    {
        __wdNr = wd;

        /* reset the watchdog */
        wd_reset( __wdNr );
        /* configure its timeout period */
        wd_config( __wdNr, timeoutTicks , WDEX_IRQ_RESET );
        /* register its ISR handler */
        wd_registerIntHandler( __wdNr, &__wdHandler );
        /* and start the watchdog */
        wd_start( __wdNr );
        /* finally assign it a high interrupt priority */
        wd_setIntPriority(0);

        retVal = pdPASS;
    }

    return retVal;
}


/**
 * A FreRTOS task that periodically reloads the configured watchdog.
 * If no watchdog has been configured, the task yields immediately.
 *
 * If 'pvParams' is NULL, the default period of 5 seconds will be applied.
 *
 * @param pvParameters - a pointer to an instance of wdDelayParams which includes the task's period in milliseconds
 */
void wdTask( void* pvParameters )
{
    const wdDelayParam* const param = (wdDelayParam*) pvParameters;
    const uint32_t delay = ( NULL==param ? APP_WD_TIMEOUT_MS/2 : param->delayMs );

    for ( ; ; )
    {
        /* reload the watchdog if it has been configured */
        if ( WD_UNSPECIFIED != __wdNr )
        {
            wd_reload( __wdNr );
        }

        /* block until the task's period expires */
        vTaskDelay( delay / portTICK_RATE_MS );
    }

    /* just in case the task somehow exits the infinite loop */
    vTaskDelete(NULL);
}

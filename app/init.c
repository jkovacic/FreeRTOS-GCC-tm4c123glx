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
 * The file contains a function that initializes the
 * MCU's peripherals.
 *
 * @author Jernej Kovacic
 */


#include "FreeRTOSConfig.h"

#include "sysctl.h"
#include "fpu.h"

/*
 * Implemented in gpio.c, sysctl.c and watchdog.c, respectively
 */
extern void _gpio_initIntHandlers(void);
extern void _sysctl_enableGpioAhb(void);
extern void _wd_initIntHandlers(void);


/*
 * An "unofficial" function (not exposed in any header)
 * that performs initialization of MCU's peripherals.
 *
 * It should only be called from the startup routine before
 * the execution is passed to a user application
 * (typically started in main().)
 */
void _init(void)
{
    /* Initializes the MCU revision number: */
    sysctl_mcuRevision();

    /* Configure system clock frequency to 50 MHz (default) */
    sysctl_configSysClock(APP_SYS_CLOCK_DIV);

    /* Depending on configuration, enable GPIO AHB mode: */
    if ( 0 != APP_GPIO_AHB )
    {
        _sysctl_enableGpioAhb();
    }


    /* Enable/disable FPU: */
    if ( 0 != APP_FPU_ENABLE )
    {
        fpu_enable();

        /* Enable/disable lazy stacking of FPU's registers */
        if ( 0 != APP_FPU_LAZY_STACKING )
        {
        	fpu_enableLazyStacking();
        }
        else
        {
            fpu_enableStacking();
        }
    }
    else
    {
        fpu_disable();
    }

    /*
     * Initialize the tables of GPIO and
     * watchdog interrupt handlers.
     */
    _gpio_initIntHandlers();
    _wd_initIntHandlers();
}

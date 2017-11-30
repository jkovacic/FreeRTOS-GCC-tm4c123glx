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
 * Declaration of public functions that handle
 * both watchdog timers.
 *
 * @author Jernej Kovacic
 */


#ifndef _WATCHDOG_H_
#define _WATCHDOG_H_


#include <stdint.h>


/**
 * An enumeration with supported exception types
 * when a watchdog counter reaches 0.
 */
typedef enum _watchdog_exception
{
    WDEX_NMI,           /** Non-maskable interrupt (NMI) */
    WDEX_IRQ,           /** Standard interrupt */
    WDEX_NMI_RESET,     /** NMI on first time-out, reset on second */
    WDEX_IRQ_RESET      /** Standard int. on first time-out, reset on second */
} WatchdogException;


/**
 * Required prototype for watchdog triggered interrupt requests.
 */
typedef void (*WatchdogIntHandler_t)(void);


void wd_enableWd(uint8_t wd);

void wd_disableWd(uint8_t wd);

void  wd_start(uint8_t wd);

void wd_reset(uint8_t wd);

void wd_enableNvicIntr(void);

void wd_disableNvicIntr(void);

void wd_setIntPriority(uint8_t pri);

void wd_clearInterrupt(uint8_t wd);

uint32_t wd_getValue(uint8_t wd);

void wd_registerIntHandler(uint8_t wd, WatchdogIntHandler_t isr);

void wd_unregisterIntHandler(uint8_t wd);

void wd_reload(uint8_t wd);

void wd_config(uint8_t wd, uint32_t loadValue, WatchdogException ex);

#endif  /* _WATCHDOG_H_ */

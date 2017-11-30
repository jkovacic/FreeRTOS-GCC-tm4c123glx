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
 * the board's System Controller (SysCtrl).
 *
 * @author Jernej Kovacic
 */


#ifndef _SYSCTL_H_
#define _SYSCTL_H_

#include <stdint.h>

int8_t sysctl_mcuRevision(void);

uint8_t sysctl_configSysClock(uint8_t div);

void sysctl_enableGpioPort(uint8_t port);

void sysctl_disableGpioPort(uint8_t port);

void sysctl_enableUart(uint8_t uartNr);

void sysctl_disableUart(uint8_t uartNr);

void sysctl_enableWatchdog(uint8_t wd);

void sysctl_disableWatchdog(uint8_t wd);

void sysctl_resetWatchdog(uint8_t wd);

#endif  /* _SYSCTL_H_ */

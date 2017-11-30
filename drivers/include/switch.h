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
 * Declaration of public functions that manipulate both
 * switches of the Texas Instruments TM4C123GLX Launchpad
 * directly, without using general GPIO functions.
 *
 * @author Jernej Kovacic
 */


#ifndef _SWITCH_H_
#define _SWITCH_H_

#include <stdint.h>

#include "gpio.h"

/**
 * Bit masks for both switches
 */

#define SWITCH1       ( 0x00000010 )
#define SWITCH2       ( 0x00000001 )


void switch_config(void);

uint8_t switch_statusSw1(void);

uint8_t switch_statusSw2(void);

uint8_t switch_statusBoth(void);

void switch_enableSwInt(uint8_t sw);

void switch_disableSwInt(uint8_t sw);

void switch_registerIntrHandler(uint8_t sw, GpioPortIntHandler_t isr);

void switch_unregisterIntrHandler(uint8_t sw);

void switch_clearIntr(uint8_t sw);

#endif  /* _SWITCH_H_ */

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
 * Declaration of data structures and functions that
 * handle switches and LEDs.
 *
 * @author Jernej Kovacic
 */

#ifndef _LIGHTSHOW_H_
#define _LIGHTSHOW_H_

#include <stdint.h>

#include <task.h>


/**
 * A struct with parameters to be passed to the
 * task that handles the switch 1.
 */
typedef struct _Switch1TaskParam_t
{
    TaskHandle_t ledHandle;    /** Handle of the light show task */
} Switch1TaskParam_t;


/**
 * A struct with parameters to be passed to the
 * light show task that periodically controls LEDs.
 */
typedef struct _LightShowParam_t
{
    uint32_t delayMs;          /** Period of LED switching in milliseconds */
} LightShowParam_t;


int16_t lightshowInit(void);

void lightshowTask(void* params);

void sw1DsrTask(void* params);

#endif  /* _LIGHTSHOW_H_ */

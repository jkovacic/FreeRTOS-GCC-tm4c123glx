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
 * Declaration of functions that handle printing via a UART.
 *
 * @author Jernej Kovacic
 */


#ifndef _PRINT_H_
#define _PRINT_H_

#include <stdint.h>

#include <FreeRTOS.h>


/**
 * A struct with parameters to be passed to the
 * task that prints messages to UART(s)
 */
typedef struct _printUartParam
{
    uint8_t uartNr;     /** UART number */
} printUartParam;


int16_t printInit(uint8_t uart_nr);

void printGateKeeperTask(void* params);

void vPrintMsg(uint8_t uart, const portCHAR* msg);

void vPrintChar(uint8_t uart, portCHAR ch);

void vDirectPrintMsg(const portCHAR* msg);

void vDirectPrintCh(portCHAR ch);


#endif  /* _PRINT_H_ */

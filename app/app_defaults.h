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
 * Sensible defaults for supported peripherals of
 * Texas Instruments TM4C123GLX Launchpad.
 *
 * @author Jernej Kovacic
 */


#ifndef _APP_DEFAULTS_H_
#define _APP_DEFAULTS_H_

#include "uart.h"


/*
 * Sensible defaults for all UART communications:
 * - 8 data bits
 * - no parity
 * - 1 stop bit
 */
#define DEF_UART_DATA_BITS      ( 8 )
#define DEF_UART_PARITY         ( PAR_NONE )
#define DEF_UART_STOP           ( 1 )


/*
 * Default settings for the UART0:
 * pins 0 and 1 of the GPIO port A, baud rate=115200
 */
#define DEF_UART0_PORT          ( GPIO_PORTA )
#define DEF_UART0_PIN_RX        ( 0 )
#define DEF_UART0_PIN_TX        ( 1 )
#define DEF_UART0_PCTL          ( 1 )
#define DEF_UART0_BR            ( BR_115200 )
#define DEF_UART0_DATA_BITS     DEF_UART_DATA_BITS
#define DEF_UART0_PARITY        DEF_UART_PARITY
#define DEF_UART0_STOP          DEF_UART_STOP

/*
 * Default settings for the UART1:
 * pins 0 and 1 of the GPIO port B, baud rate=115200
 */
#define DEF_UART1_PORT          ( GPIO_PORTB )
#define DEF_UART1_PIN_RX        ( 0 )
#define DEF_UART1_PIN_TX        ( 1 )
#define DEF_UART1_PCTL          ( 1 )
#define DEF_UART1_BR            ( BR_115200 )
#define DEF_UART1_DATA_BITS     DEF_UART_DATA_BITS
#define DEF_UART1_PARITY        DEF_UART_PARITY
#define DEF_UART1_STOP          DEF_UART_STOP


#endif  /* _APP_DEFAULTS_H_ */

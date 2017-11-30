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
 * the board's UART controllers.
 *
 * @author Jernej Kovacic
 */

#ifndef _UART_H_
#define _UART_H_

#include <stdint.h>


/**
 * An enumeration with supported UART baud rates.
 */
typedef enum _baud_rate_t
{
    BR_9600,
    BR_19200,
    BR_38400,
    BR_57600,
    BR_115200
} baud_rate_t;


/**
 * An enumeration with supported parities
 */
typedef enum _parity_t
{
    PAR_NONE,       /** no parity */
    PAR_ODD,        /** odd parity */
    PAR_EVEN,       /** even parity */
    PAR_STICKY_0,   /** transmits the parity bit and check as 0 */
    PAR_STICKY_1    /** transmits the parity bit and check as 1 */
} parity_t;


/**
 * An enum with possible levels when Rx interrupt
 * is triggered
 */
typedef enum _rx_interrupt_fifo_level_t
{
    RXFIFO_1_8_FULL,      /** Rx FIFO >= 1/8 full */
    RXFIFO_1_4_FULL,      /** Rx FIFO >= 1/4 full */
    RXFIFO_1_2_FULL,      /** Rx FIFO >= 1/2 full */
    RXFIFO_3_4_FULL,      /** Rx FIFO >= 3/4 full */
    RXFIFO_7_8_FULL       /** Rx FIFO >= 7/8 full */
} rx_interrupt_fifo_level_t;



void uart_enableUart(uint8_t nr);

void uart_disableUart(uint8_t nr);

void uart_flushTxFifo(uint8_t nr);

void uart_enableRx(uint8_t nr);

void uart_disableRx(uint8_t nr);

void uart_enableTx(uint8_t nr);

void uart_disableTx(uint8_t nr);

void uart_enableRxIntr(uint8_t nr);

void uart_disableRxIntr(uint8_t nr);

void uart_clearRxIntr(uint8_t nr);

void uart_characterMode(uint8_t nr);

void uart_fifoMode(uint8_t nr, rx_interrupt_fifo_level_t level);

void uart_enableNvicIntr(uint8_t nr);

void uart_disableNvicIntr(uint8_t nr);

void uart_setIntrPriority(uint8_t nr, uint8_t pri);

char uart_readChar(uint8_t nr);

void uart_printStr(uint8_t nr, const char* str);

void uart_printCh(uint8_t nr, char ch);

void uart_config(
        uint8_t nr,
        uint8_t gp,
        uint8_t pinRx,
        uint8_t pinTx,
        uint8_t pctl,
        baud_rate_t br,
        uint8_t data_bits,
        parity_t parity,
        uint8_t stop );

#endif   /* _UART_H_ */

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
 * Implementation of exception and interrupt handler functions.
 *
 * @author Jernej Kovacic
 */


#include <stdint.h>

#include "gpio.h"


/* forward declaration of an "unofficial" function from gpio.c */
extern void _gpio_intHandler(uint8_t port);

/* forward declaration of an "unofficial" function from receive.c */
extern void _recv_intHandler(uint8_t uart);


/*
 * Handler for interrupts, triggered by a pin
 * from the GPIO port A.
 */
__attribute__ ((interrupt))
void GpioAIntHandler(void)
{
    _gpio_intHandler(GPIO_PORTA);
}


/*
 * Handler for interrupts, triggered by a pin
 * from the GPIO port B.
 */
__attribute__ ((interrupt))
void GpioBIntHandler(void)
{
    _gpio_intHandler(GPIO_PORTB);
}


/*
 * Handler for interrupts, triggered by a pin
 * from the GPIO port C.
 */
__attribute__ ((interrupt))
void GpioCIntHandler(void)
{
    _gpio_intHandler(GPIO_PORTC);
}


/*
 * Handler for interrupts, triggered by a pin
 * from the GPIO port D.
 */
__attribute__ ((interrupt))
void GpioDIntHandler(void)
{
    _gpio_intHandler(GPIO_PORTD);
}


/*
 * Handler for interrupts, triggered by a pin
 * from the GPIO port E.
 */
__attribute__ ((interrupt))
void GpioEIntHandler(void)
{
    _gpio_intHandler(GPIO_PORTE);
}


/*
 * Handler for interrupts, triggered by a pin
 * from the GPIO port F.
 */
__attribute__ ((interrupt))
void GpioFIntHandler(void)
{
    _gpio_intHandler(GPIO_PORTF);
}



/*
 * Handler for interrupts, triggered by UART0.
 */
__attribute__ ((interrupt))
void Uart0IntHandler(void)
{
    _recv_intHandler(0);
}


/*
 * Handler for interrupts, triggered by UART1.
 */
__attribute__ ((interrupt))
void Uart1IntHandler(void)
{
    _recv_intHandler(1);
}


/*
 * Handler for interrupts, triggered by UART2.
 */
__attribute__ ((interrupt))
void Uart2IntHandler(void)
{
    _recv_intHandler(2);
}


/*
 * Handler for interrupts, triggered by UART3.
 */
__attribute__ ((interrupt))
void Uart3IntHandler(void)
{
    _recv_intHandler(3);
}


/*
 * Handler for interrupts, triggered by UART4.
 */
__attribute__ ((interrupt))
void Uart4IntHandler(void)
{
    _recv_intHandler(4);
}


/*
 * Handler for interrupts, triggered by UART5.
 */
__attribute__ ((interrupt))
void Uart5IntHandler(void)
{
    _recv_intHandler(5);
}


/*
 * Handler for interrupts, triggered by UART6.
 */
__attribute__ ((interrupt))
void Uart6IntHandler(void)
{
    _recv_intHandler(6);
}


/*
 * Handler for interrupts, triggered by UART7.
 */
__attribute__ ((interrupt))
void Uart7IntHandler(void)
{
    _recv_intHandler(7);
}

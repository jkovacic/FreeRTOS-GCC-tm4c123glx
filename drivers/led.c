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
 * Implementation of functions that manipulate all 3 LEDs of
 * the Texas Instruments TM4C123GLX Launchpad directly,
 * without using general GPIO functions. This is an intermediate
 * layer, between a developer and appropriate GPIO handling functions
 * and no knowledge of LEDs' technical details is required.
 *
 * More details about the Launchpad, see
 * Tiva(TM) C Series TM4C123G LaunchPad Evaluation Board User's Guide
 * available at:
 * http://www.ti.com/lit/ug/spmu296/spmu296.pdf
 *
 * @author Jernej Kovacic
 */

#include <stdint.h>

#include "led.h"
#include "gpio.h"



/*
 * As evident from the page 9 of the User's Guide, the LEDs
 * are connected to the following GPIO pins:
 * - PF1: red LED
 * - PF2: blue LED
 * - PF3: green LED
 */

/* The GPIO port, all 3 LEDS are connected to: */
#define PORT_LED      ( GPIO_PORTF )

/* Bit mask of pins connected to all 3 LEDs: */
#define LED_MASK      ( LED_RED | LED_GREEN | LED_BLUE )

/* Pin numbers for all LEDs */
#define PIN_RED       ( 1 )
#define PIN_BLUE      ( 2 )
#define PIN_GREEN     ( 3 )

/*
 * Configures all 3 GPIO pins, connected to LEDs.
 */
void led_config(void)
{
    /*
     * All 3 pins are configured as regular output GPIO pins.
     */
    gpio_configGpioPin(PORT_LED, PIN_RED, 1);
    gpio_configGpioPin(PORT_LED, PIN_BLUE, 1);
    gpio_configGpioPin(PORT_LED, PIN_GREEN, 1);
}


/**
 * Turns all 3 LEDs off.
 */
void led_allOff(void)
{
    led_off(LED_MASK);
}


/**
 * Turn the selected LED(s) on.
 *
 * It is possible to turn on several LEDs simultaneously.
 * If this is desired, just compose 'leds' by bitwise ORing
 * of all desired LEDs' bit masks, e.g.:
 *
 *     led = LED_RED | LED_GREEN;
 *
 * @param leds - bit mask of selected LEDs.
 */
void led_on(uint32_t leds)
{
    gpio_setPins(PORT_LED, leds & LED_MASK);
}


/**
 * Turn the selected LED(s) off.
 *
 * It is possible to turn off several LEDs simultaneously.
 * If this is desired, just compose 'leds' by bitwise ORing
 * of all desired LEDs' bit masks, e.g.:
 *
 *     led = LED_RED | LED_GREEN;
 *
 * @param leds - bit mask of selected LEDs.
 */
void led_off(uint32_t leds)
{
    gpio_clearPins(PORT_LED, leds & LED_MASK);
}

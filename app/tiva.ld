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
 * A linker script that defines the device's memory layout and determines
 * where each application's section is placed.
 *
 * To verify whether objects (symbols) have been placed to expected locations, run
 * 'nm' ('arm-none-eabi-nm') on the resulting ELF image.
 */


/*
 * Memory map origin and size of the built-in memory.
 */
MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00040000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000
}


/*
 * This part of the linker script defines where each application's
 * section will be placed. .text (the actual code) and .rodata 
 * (constant global variables) remain unmodified during an application
 * run, so they will be placed into flash. On the other hand, .data
 * (initialized non-constant global variables) and .bss (non-initialized
 * non-constant global variables) will be placed into SRAM.
 *
 * Note> stack space has been allocated as an array in startup.c, so the
 * stack pointer is defined there.
 */
SECTIONS
{
    .text :
    {
        _text = .;
        
        /*
         * The vector table must be placed to the top of the
         * memory map. To achieve this, it was assigned to a 
         * special section called ".isr_vector"
         */
        KEEP(*(.isr_vector))
        
        /* followed by .text and .rodata: */
        *(.text*)
        *(.rodata*)
        
        _etext = .;
    } > FLASH

    /* Just to make sure that the contents does not exceed the flash size */
    . = ORIGIN(FLASH) + LENGTH(FLASH);
    
    
    /*
     * .data and .bss are placed into SRAM:
     */
    .data : AT(ADDR(.text) + SIZEOF(.text))
    {
        _data = .;
        *(.data*)
        _edata = .;
    } > SRAM

    .bss :
    {
        /* _bss and _ebss will be required during initialization */
        _bss = .;
        *(.bss*)
        _ebss = .;
    } > SRAM
    
    /* Just to make sure that the contents does not exceed the SRAM size */
    . = ORIGIN(SRAM) + LENGTH(SRAM);
}

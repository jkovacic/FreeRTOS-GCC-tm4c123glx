/*
 * port.c from the officially supported GCC/ARM_CM4F port was reused for
 * TI TM4C123G*. The following changes were applied:
 *
 * - some stuff, already implemented by generic drivers, was removed from this file
 * - vPortSetupTimerInterrupt() was modified to use generic SysTick drivers
 * - context switching, implemented in xPortPendSVHandler(), also unpends the PendSV exception
 * - context switching, implemented in xPortPendSVHandler(), also supports situations when
 *   FPU and stacking its registers are disabled
 * - configuration of FPU and stacking its registers was removed from this file as
 *   it is implemented in the startup routine
 * - assembler instructions were rewritten in upper case for better readability
 * - all "annoying" tabs have been replaced by spaces
 *
 * The original file is available under the following license:
 */

/*
 * FreeRTOS Kernel V10.4.0
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */


/*-----------------------------------------------------------
 * Implementation of functions defined in portable.h for the ARM CM4F port.
 *----------------------------------------------------------*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "systick.h"
#include "scb.h"
#include "fpu.h"

/*
 * When configuring the PRIMASK register, the desired priority
 * mask (between 0 and 7) must be shifted by this value.
 * This is specific for TI TM4C123G* processors.
 */
#define PRIMASK_SHIFT               ( 5 )

/* Constants required to set up the initial stack. */
#define portINITIAL_XPSR            ( 0x01000000 )
#define portINITIAL_EXC_RETURN      ( 0xfffffffd )


/*
 * For strict compliance with the Cortex-M spec the task start address should
 * have bit-0 clear, as it is loaded into the PC on exit from an ISR.
 */
#define portSTART_ADDRESS_MASK      ( ( StackType_t ) 0xfffffffeUL )

/*
 * A fiddle factor to estimate the number of SysTick counts that would have
 * occurred while the SysTick counter is stopped during tickless idle
 * calculations.
 */
#define portMISSED_COUNTS_FACTOR    ( 45UL )

/*
 * Let the user override the pre-loading of the initial LR with the address of
 * prvTaskExitError() in case it messes up unwinding of the stack in the
 * debugger.
 */
#ifdef configTASK_RETURN_ADDRESS
    #define portTASK_RETURN_ADDRESS	    configTASK_RETURN_ADDRESS
#else
    #define portTASK_RETURN_ADDRESS	    prvTaskExitError
#endif

/*
 * Setup the timer to generate the tick interrupts.  The implementation in this
 * file is weak to allow application writers to change the timer used to
 * generate the tick interrupt.
 */
void vPortSetupTimerInterrupt( void );


/*
 * Exception handlers.
 */
void xPortPendSVHandler( void ) __attribute__ (( naked ));
void xPortSysTickHandler( void ) __attribute__(( interrupt ));
void vPortSVCHandler( void ) __attribute__ (( naked ));

/*
 * Start first task is a separate function so it can be tested in isolation.
 */
static void prvPortStartFirstTask( void ) __attribute__ (( naked ));


/*
 * Used to catch tasks that attempt to return from their implementing function.
 */
static void prvTaskExitError( void );

/*-----------------------------------------------------------*/

/* Each task maintains its own interrupt status in the critical nesting
variable. */
static UBaseType_t uxCriticalNesting = 0xaaaaaaaa;

/*
 * The number of SysTick increments that make up one tick period.
 */
#if( configUSE_TICKLESS_IDLE == 1 )
    static uint32_t ulTimerCountsForOneTick = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * 24 bit resolution of the SysTick timer.
 */
#if( configUSE_TICKLESS_IDLE == 1 )
    static uint32_t xMaximumPossibleSuppressedTicks = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Compensate for the CPU cycles that pass while the SysTick is stopped (low
 * power functionality only.
 */
#if( configUSE_TICKLESS_IDLE == 1 )
    static uint32_t ulStoppedTimerCompensation = 0;
#endif /* configUSE_TICKLESS_IDLE */

/*
 * Used by the portASSERT_IF_INTERRUPT_PRIORITY_INVALID() macro to ensure
 * FreeRTOS API functions are not called from interrupts that have been assigned
 * a priority above configMAX_SYSCALL_INTERRUPT_PRIORITY.
 */
#if( configASSERT_DEFINED == 1 )
    static uint8_t ucMaxSysCallPriority = 0;
    static uint32_t ulMaxPRIGROUPValue = 0;
    static const volatile uint8_t * const pcInterruptPriorityRegisters = ( const volatile uint8_t * const ) portNVIC_IP_REGISTERS_OFFSET_16;
#endif /* configASSERT_DEFINED */

/*-----------------------------------------------------------*/


/*
 * See header file for description.
 */
StackType_t* pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
    /*
     * Simulate the stack frame as it would be created by a
     * context switch interrupt.
     */

    /*
     * Offset added to account for the way the MCU uses the stack on
     * entry/exit of interrupts, and to ensure alignment.
     */
    pxTopOfStack--;

    *pxTopOfStack = portINITIAL_XPSR;             /* xPSR */
    pxTopOfStack--;
    *pxTopOfStack = ( ( StackType_t ) pxCode ) & portSTART_ADDRESS_MASK;       /* PC */
    pxTopOfStack--;
    *pxTopOfStack = ( StackType_t ) portTASK_RETURN_ADDRESS;  /* LR */

    /* Save code space by skipping register initialisation. */
    pxTopOfStack -= 5;                   /* R12, R3, R2 and R1. */
    *pxTopOfStack = ( StackType_t ) pvParameters;   /* R0 */

    /*
     * A save method is being used that requires each task to
     * maintain its own exec return value.
     */
    pxTopOfStack--;
    *pxTopOfStack = portINITIAL_EXC_RETURN;

    pxTopOfStack -= 8;   /* R11, R10, R9, R8, R7, R6, R5 and R4. */

    return pxTopOfStack;
}
/*-----------------------------------------------------------*/

static void prvTaskExitError( void )
{
    volatile uint32_t ulDummy = 0;

    /*
     * A function that implements a task must not exit or attempt to
     * return to its caller as there is nothing to return to.  If a
     * task wants to exit it should instead call vTaskDelete( NULL ).
     *
     * Artificially force an assert() to be triggered if configASSERT()
     * is defined, then stop here so application writers can catch the error.
     */
    configASSERT( uxCriticalNesting == ~0UL );
    portDISABLE_INTERRUPTS();
    while( ulDummy == 0 )
    {
        /* This file calls prvTaskExitError() after the scheduler has been
           started to remove a compiler warning about the function being defined
            but never called.  ulDummy is used purely to quieten other warnings
            about code appearing after this function is called - making ulDummy
            volatile makes the compiler think the function could return and
            therefore not output an 'unreachable code' warning for code that appears
            after it. */
    }
}
/*-----------------------------------------------------------*/


void vPortSVCHandler( void )
{
    __asm volatile (
        "    LDR    r3, pxCurrentTCBConst2       \n" /* Restore the context. */
        "    LDR    r1, [r3]                     \n" /* Use pxCurrentTCBConst to get the pxCurrentTCB address. */
        "    LDR    r0, [r1]                     \n" /* The first item in pxCurrentTCB is the task top of stack. */
        "    LDMIA  r0!, {r4-r11, r14}           \n" /* Pop the registers that are not automatically saved on exception entry and the critical nesting count. */
        "    MSR    psp, r0                      \n" /* Restore the task stack pointer. */
        "    ISB                                 \n"
        "    MOV    r0, #0                       \n"
        "    MSR    basepri, r0                  \n"
        "    BX     r14                          \n"
        "                                        \n"
        "    .align 4                            \n"
        "pxCurrentTCBConst2:  .word pxCurrentTCB \n"
   );
}
/*-----------------------------------------------------------*/


static void prvPortStartFirstTask( void )
{
    /* Start the first task.  This also clears the bit that indicates the FPU is
       in use in case the FPU was used before the scheduler was started - which
        would otherwise result in the unnecessary leaving of space in the SVC stack
        for lazy saving of FPU registers. */

    __asm volatile(
        " LDR     r0, =0xE000ED08      \n" /* Use the NVIC offset register to locate the stack. */
        " LDR     r0, [r0]             \n"
        " LDR     r0, [r0]             \n"
        " MSR     msp, r0              \n" /* Set the msp back to the start of the stack. */
        " MOV     r0, #0               \n" /* Clear the bit that indicates the FPU is in use, see comment above. */
        " MSR     control, r0          \n"
        " CPSIE   i                    \n" /* Globally enable interrupts. */
        " CPSIE   f                    \n" /* Globally enable fault handlers */
        " DSB                          \n"
        " ISB                          \n"
        " SVC     0                    \n" /* System call to start first task. */
        " NOP                          \n"
        " .ltorg				       \n"
    );
}
/*-----------------------------------------------------------*/


/*
 * See header file for description.
 */
BaseType_t xPortStartScheduler( void )
{
    /*
     * configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to 0.
     * See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html
     */
    configASSERT( configMAX_SYSCALL_INTERRUPT_PRIORITY << PRIMASK_SHIFT );

    #if( configASSERT_DEFINED == 1 )
    {
        volatile uint32_t ulOriginalPriority;
        volatile uint8_t * const pucFirstUserPriorityRegister = ( volatile uint8_t * const ) ( portNVIC_IP_REGISTERS_OFFSET_16 + portFIRST_USER_INTERRUPT_NUMBER );
        volatile uint8_t ucMaxPriorityValue;

        /*
         * Determine the maximum priority from which ISR safe FreeRTOS API
         * functions can be called.  ISR safe functions are those that end in
         * "FromISR".  FreeRTOS maintains separate thread and ISR API functions to
         * ensure interrupt entry is as fast and simple as possible.

         * Save the interrupt priority value that is about to be clobbered.
         */
        ulOriginalPriority = *pucFirstUserPriorityRegister;

        /*
         * Determine the number of priority bits available.  First write to
         * all possible bits.
         */
        *pucFirstUserPriorityRegister = portMAX_8_BIT_VALUE;

        /* Read the value back to see how many bits stuck. */
        ucMaxPriorityValue = *pucFirstUserPriorityRegister;

        /* Use the same mask on the maximum system call priority. */
        ucMaxSysCallPriority = (configMAX_SYSCALL_INTERRUPT_PRIORITY << PRIMASK_SHIFT) & ucMaxPriorityValue;

        /* Calculate the maximum acceptable priority group value for the
         * number of bits read back. */
        ulMaxPRIGROUPValue = portMAX_PRIGROUP_BITS;
        while( ( ucMaxPriorityValue & portTOP_BIT_OF_BYTE ) == portTOP_BIT_OF_BYTE )
        {
            ulMaxPRIGROUPValue--;
            ucMaxPriorityValue <<= ( uint8_t ) 0x01;
        }

        #ifdef __NVIC_PRIO_BITS
        {
            /* Check the CMSIS configuration that defines the number of
               priority bits matches the number of priority bits actually queried
               from the hardware. */
            configASSERT( ( portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue ) == __NVIC_PRIO_BITS );
        }
        #endif

        #ifdef configPRIO_BITS
        {
            /* Check the FreeRTOS configuration that defines the number of
               priority bits matches the number of priority bits actually queried
               from the hardware. */
            configASSERT( ( portMAX_PRIGROUP_BITS - ulMaxPRIGROUPValue ) == configPRIO_BITS );
        }
        #endif

        /*
         * Shift the priority group value back to its position within
         * the AIRCR register.
         */
        ulMaxPRIGROUPValue <<= portPRIGROUP_SHIFT;
        ulMaxPRIGROUPValue &= portPRIORITY_GROUP_MASK;

        /*
         * Restore the clobbered interrupt priority register
         * to its original value. */
        *pucFirstUserPriorityRegister = ulOriginalPriority;
    }
    #endif /* conifgASSERT_DEFINED */

    /* Make PendSV and SysTick the lowest priority interrupts. */
    scb_setSysTickPriority( configKERNEL_INTERRUPT_PRIORITY  );
    scb_setPendSvPriority( configKERNEL_INTERRUPT_PRIORITY  );

    /*
     * Note that the FPU has already been configured in _init().
     */

    /* Start the timer that generates the tick ISR.  Interrupts
     * are disabled here already.
     */
    vPortSetupTimerInterrupt();

    /* Initialise the critical nesting count ready for the first task. */
    uxCriticalNesting = 0;

    /* Start the first task. */
    prvPortStartFirstTask();

    /*
     * Should never get here as the tasks will now be executing!
     * Call the task exit error function to prevent compiler warnings
     * about a static function not being called in the case that the
     * application writer overrides this functionality by defining
     * configTASK_RETURN_ADDRESS. Call
     * vTaskSwitchContext() so link time optimisation does not remove the
     * symbol.
     */
    vTaskSwitchContext();
    prvTaskExitError();

    /* Should not get here! */
    return 0;
}
/*-----------------------------------------------------------*/

void vPortEndScheduler( void )
{
    /*
     * Not implemented in ports where there is nothing
     * to return to. Artificially force an assert.
     */
    configASSERT( uxCriticalNesting == 1000UL );
}
/*-----------------------------------------------------------*/


void vPortEnterCritical( void )
{
    portDISABLE_INTERRUPTS();
    uxCriticalNesting++;

    /*
     * This is not the interrupt safe version of the enter critical function so
     * assert() if it is being called from an interrupt context.  Only API
     * functions that end in "FromISR" can be used in an interrupt.  Only assert if
     * the critical nesting count is 1 to protect against recursive calls if the
     * assert function also uses a critical section.
     */
    if( uxCriticalNesting == 1 )
    {
         configASSERT( ( scb_activeException() ) == 0 );
    }
}
/*-----------------------------------------------------------*/

void vPortExitCritical( void )
{
    configASSERT( uxCriticalNesting );
    uxCriticalNesting--;
    if( uxCriticalNesting == 0 )
    {
        portENABLE_INTERRUPTS();
    }
}


/*-----------------------------------------------------------*/


void xPortPendSVHandler( void )
{
    /* This is a naked function. */

    __asm volatile
    (
    "   MRS       r0, psp                   \n"
    "   ISB                                 \n"
    "                                       \n"
    "   LDR       r3, pxCurrentTCBConst     \n" /* Get the location of the current TCB. */
    "   LDR       r2, [r3]                  \n"
    "                                       \n"
#if APP_FPU_ENABLE != 0
    "   TST       r14, #0x10                \n" /* Is the task using the FPU context?  If so, push high vfp registers. */
    "   IT        eq                        \n"
    "   VSTMDBEQ  r0!, {s16-s31}            \n"
    "                                       \n"
#endif  /* APP_FPU_ENABLE */
    "   STMDB     r0!, {r4-r11, r14}        \n" /* Save the core registers. */
    "   STR       r0, [r2]                  \n" /* Save the new top of stack into the first member of the TCB. */
    "                                       \n"
    "   STMDB     sp!, {r0, r3}             \n"
    "   MOV       r0, %0                    \n"
    "   MSR       basepri, r0               \n"
    "   DSB                                 \n"
    "   ISB                                 \n"
    "   BL        scb_clearPendSv           \n" /* unpend PendSV exception */
    "   BL        vTaskSwitchContext        \n"
    "   MOV       r0, #0                    \n"
    "   MSR       basepri, r0               \n"
    "   LDMIA     sp!, {r0, r3}             \n"
    "                                       \n"
    "   LDR       r1, [r3]                  \n" /* The first item in pxCurrentTCB is the task top of stack. */
    "   LDR       r0, [r1]                  \n"
    "                                       \n"
    "   LDMIA     r0!, {r4-r11, r14}	    \n" /* Pop the core registers. */
    "                                       \n"
#if APP_FPU_ENABLE != 0
    "   TST       r14, #0x10                \n" /* Is the task using the FPU context?  If so, pop the high vfp registers too. */
    "   IT        eq                        \n"
    "   VLDMIAEQ  r0!, {s16-s31}            \n"
    "                                       \n"
#endif  /* APP_FPU_ENABLE */
    "   MSR       psp, r0                   \n"
    "   ISB                                 \n"
    "                                       \n"
    "                                       \n"
    "   BX        r14                       \n"
    "                                       \n"
    "   .align    4                         \n"
    "pxCurrentTCBConst: .word pxCurrentTCB	\n"
    ::"i"(configMAX_SYSCALL_INTERRUPT_PRIORITY << PRIMASK_SHIFT )
    );
}
/*-----------------------------------------------------------*/


void xPortSysTickHandler( void )
{
    /*
     * The SysTick runs at the lowest interrupt priority, so when
     * this interrupt executes all interrupts must be unmasked.
     * There is therefore no need to save and then restore the interrupt
     * mask value as its value is already known.
     */

	portDISABLE_INTERRUPTS();
    {
        /* Increment the RTOS tick. */
        if( xTaskIncrementTick() != pdFALSE )
        {
            /*
             * A context switch is required.  Context switching
             * is performed in the PendSV interrupt.  Pend the
             * PendSV interrupt.
             */
            scb_triggerPendSv();
        }
    }
    portENABLE_INTERRUPTS();
}
/*-----------------------------------------------------------*/


#if( configUSE_TICKLESS_IDLE == 1 )

    __attribute__((weak)) void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
    {
        uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements;
        TickType_t xModifiableIdleTime;

        /* Make sure the SysTick reload value does not overflow the counter. */
        if( xExpectedIdleTime > xMaximumPossibleSuppressedTicks )
        {
            xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
        }

        /*
         * Stop the SysTick momentarily.  The time the SysTick is stopped for
         * is accounted for as best it can be, but using the tickless mode will
         * inevitably result in some tiny drift of the time maintained by the
         * kernel with respect to calendar time.
         */
        portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

        /*
         * Calculate the reload value required to wait xExpectedIdleTime
         * tick periods.  -1 is used because this code will execute part way
         * through one of the tick periods.
         */
        ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG + ( ulTimerCountsForOneTick * ( xExpectedIdleTime - 1UL ) );
        if( ulReloadValue > ulStoppedTimerCompensation )
        {
            ulReloadValue -= ulStoppedTimerCompensation;
        }

        /*
         * Enter a critical section but don't use the taskENTER_CRITICAL()
         * method as that will mask interrupts that should exit sleep mode.
         */
        __asm volatile( "CPSID i" ::: "memory");
        __asm volatile( "DSB" );
        __asm volatile( "ISB" );

        /*
         * If a context switch is pending or a task is waiting for the scheduler
         * to be unsuspended then abandon the low power entry.
         */
        if( eTaskConfirmSleepModeStatus() == eAbortSleep )
        {
            /*
             * Restart from whatever is left in the count register
             * to complete this tick period.
             */
            portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;

            /* Restart SysTick. */
            portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

            /*
             * Reset the reload register to the value required for
             * normal tick periods.
             */
            portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

            /*
             * Re-enable interrupts - see comments above the cpsid
             * instruction() above.
             */
            __asm volatile( "CPSIE i" ::: "memory" );
        }
        else
        {
            /* Set the new reload value. */
            portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

            /*
             * Clear the SysTick count flag and set the count value
             * back to zero.
             */
            portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

            /* Restart SysTick. */
            portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

            /*
             * Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
             * set its parameter to 0 to indicate that its implementation contains
             * its own wait for interrupt or wait for event instruction, and so wfi
             * should not be executed again.  However, the original expected idle
             * time variable must remain unmodified, so a copy is taken.
             */
            xModifiableIdleTime = xExpectedIdleTime;
            configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
            if( xModifiableIdleTime > 0 )
            {
                __asm volatile( "DSB" ::: "memory" );
                __asm volatile( "WFI" );
                __asm volatile( "ISB" );
            }
            configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

            /*
             * Re-enable interrupts to allow the interrupt that brought the MCU
             * out of sleep mode to execute immediately.  see comments above
             * __disable_interrupt() call above.
             */
                __asm volatile( "CPSIE i" ::: "memory" );
                __asm volatile( "DSB" );
                __asm volatile( "ISB" );

            /*
             * Disable interrupts again because the clock is about to be stopped
             * and interrupts that execute while the clock is stopped will increase
             * any slippage between the time maintained by the RTOS and calendar
             * time.
             */
                __asm volatile( "CPSID i" ::: "memory" );
                __asm volatile( "DSB" );
                __asm volatile( "ISB" );

            /*
             * Disable the SysTick clock without reading the
             * ortNVIC_SYSTICK_CTRL_REG register to ensure the
             * portNVIC_SYSTICK_COUNT_FLAG_BIT is not cleared if it is set.  Again,
             * the time the SysTick is stopped for is accounted for as best it can
             * be, but using the tickless mode will inevitably result in some tiny
             * drift of the time maintained by the kernel with respect to calendar
             * time
             */
            portNVIC_SYSTICK_CTRL_REG = ( portNVIC_SYSTICK_CLK_BIT | portNVIC_SYSTICK_INT_BIT );

            /*
             * Determine if the SysTick clock has already counted to zero and
             * been set back to the current reload value (the reload back being
             * correct for the entire expected idle time) or if the SysTick is yet
             * to count to zero (in which case an interrupt other than the SysTick
             * must have brought the system out of sleep mode).
             */
            if ( ( portNVIC_SYSTICK_CTRL_REG & portNVIC_SYSTICK_COUNT_FLAG_BIT ) != 0 )
            {
                uint32_t ulCalculatedLoadValue;

                /*
                 * The tick interrupt is already pending, and the SysTick count
                 * reloaded with ulReloadValue.  Reset the
                 * portNVIC_SYSTICK_LOAD_REG with whatever remains of this tick
                 * period.
                 */
                ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL ) - ( ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG );

                /*
                 * Don't allow a tiny value, or values that have somehow
                 * underflowed because the post sleep hook did something
                 * that took too long. */
                if( ( ulCalculatedLoadValue < ulStoppedTimerCompensation ) || ( ulCalculatedLoadValue > ulTimerCountsForOneTick ) )
                {
                    ulCalculatedLoadValue = ( ulTimerCountsForOneTick - 1UL );
                }

                portNVIC_SYSTICK_LOAD_REG = ulCalculatedLoadValue;

                /*
                 * As the pending tick will be processed as soon as this
                 * As function exits, the tick value maintained by the tick is stepped
                 * forward by one less than the time spent waiting.
                 */
                ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
            }
            else
            {
                /*
                 * Something other than the tick interrupt ended the sleep.
                 * Work out how long the sleep lasted rounded to complete tick
                 * periods (not the ulReload value which accounted for part
                 * ticks).
                 */
                ulCompletedSysTickDecrements = ( xExpectedIdleTime * ulTimerCountsForOneTick ) - portNVIC_SYSTICK_CURRENT_VALUE_REG;

                /*
                 * How many complete tick periods passed while
                 * the processor was waiting?
                 */
                ulCompleteTickPeriods = ulCompletedSysTickDecrements / ulTimerCountsForOneTick;

                /*
                 * The reload value is set to whatever fraction of a single
                 * tick period remains.
                 */
                portNVIC_SYSTICK_LOAD_REG = ( ( ulCompleteTickPeriods + 1UL ) * ulTimerCountsForOneTick ) - ulCompletedSysTickDecrements;
            }

            /*
             * Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
             * again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
             * value.
             */
            portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
            portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
            vTaskStepTick( ulCompleteTickPeriods );
            portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

            /* Exit with interrupts enabled. */
            __asm volatile( "CPSIE i" ::: "memory" );
        }
    }

#endif /* #if configUSE_TICKLESS_IDLE */
/*-----------------------------------------------------------*/


/*
 * Setup the systick timer to generate the tick interrupts at the required
 * frequency.
 */
__attribute__(( weak ))
void vPortSetupTimerInterrupt( void )
{
    /* Calculate the constants required to configure the tick interrupt. */
    #if( configUSE_TICKLESS_IDLE == 1 )
    {
        ulTimerCountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );
        xMaximumPossibleSuppressedTicks = portMAX_24_BIT_NUMBER / ulTimerCountsForOneTick;
        ulStoppedTimerCompensation = portMISSED_COUNTS_FACTOR / ( configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ );
    }
    #endif /* configUSE_TICKLESS_IDLE */

    /* Stop the SysTick. */
    systick_disable();

    /* Configure the SysTick and enable triggering of interrupts */
    systick_config(( configCPU_CLOCK_HZ / configTICK_RATE_HZ ) - 1UL);
    systick_enableInterrupt();

    /*
     * Start the timer.
     */
    systick_enable();
}
/*-----------------------------------------------------------*/


#if( configASSERT_DEFINED == 1 )

    void vPortValidateInterruptPriority( void )
    {
        uint32_t ulCurrentInterrupt;
        uint8_t ucCurrentPriority;

        /* Obtain the number of the currently executing interrupt. */
        __asm volatile( "MRS   %0, ipsr" : "=r"( ulCurrentInterrupt ) :: "memory" );

        /* Is the interrupt number a user defined interrupt? */
        if( ulCurrentInterrupt >= portFIRST_USER_INTERRUPT_NUMBER )
        {
            /* Look up the interrupt's priority. */
            ucCurrentPriority = pcInterruptPriorityRegisters[ ulCurrentInterrupt ];

            /*
             * The following assertion will fail if a service routine (ISR) for
             * an interrupt that has been assigned a priority above
             * configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API
             * function.  ISR safe FreeRTOS API functions must *only* be called
             * from interrupts that have been assigned a priority at or below
             * configMAX_SYSCALL_INTERRUPT_PRIORITY.
             *
             * Numerically low interrupt priority numbers represent logically high
             * interrupt priorities, therefore the priority of the interrupt must
             * be set to a value equal to or numerically *higher* than
             * configMAX_SYSCALL_INTERRUPT_PRIORITY.
             *
             * Interrupts that	use the FreeRTOS API must not be left at their
             * default priority of	zero as that is the highest possible priority,
             * which is guaranteed to be above configMAX_SYSCALL_INTERRUPT_PRIORITY,
             * and	therefore also guaranteed to be invalid.
             *
             * FreeRTOS maintains separate thread and ISR API functions to ensure
             * interrupt entry is as fast and simple as possible.
             * The following links provide detailed information:
             * http://www.freertos.org/RTOS-Cortex-M3-M4.html
             * http://www.freertos.org/FAQHelp.html
             */

            configASSERT( ucCurrentPriority >= ucMaxSysCallPriority );
        }

        /*
         * Priority grouping:  The interrupt controller (NVIC) allows the bits
         * that define each interrupt's priority to be split between bits that
         * define the interrupt's pre-emption priority bits and bits that define
         * the interrupt's sub-priority.  For simplicity all bits must be defined
         * to be pre-emption priority bits.  The following assertion will fail if
         * this is not the case (if some bits represent a sub-priority).
         *
         * If the application only uses CMSIS libraries for interrupt
         * configuration then the correct setting can be achieved on all Cortex-M
         * devices by calling NVIC_SetPriorityGrouping( 0 ); before starting the
         * scheduler.  Note however that some vendor specific peripheral libraries
         * assume a non-zero priority group setting, in which cases using a value
         * of zero will result in unpredictable behaviour.
         */

        configASSERT( ( portAIRCR_REG & portPRIORITY_GROUP_MASK ) <= ulMaxPRIGROUPValue );
    }

#endif /* configASSERT_DEFINED */

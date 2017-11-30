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
 * the Floating-Point Unit (FPU).
 *
 * @author Jernej Kovacic
 */


#ifndef _FPU_H_
#define _FPU_H_

#include <stdbool.h>


/**
 * An enumeration with supported FPU Half Precision Modes.
 */
typedef enum _FpuHalfPrecisionMode
{
    FPUHPM_IEEE,               /** IEEE representation */
    FPUHPM_ALTERNATIVE         /** Cortex-M alternative representation */
} FpuHalfPrecisionMode;


/**
 * An enumeration with supported NaN modes.
 */
typedef enum _FpuNanMode
{
    FPU_NAN_PROPAGATE,        /** IEEE representation */
    FPU_NAN_DEFAULT           /** Cortex-M alternative representation */
} FpuNanMode;


/**
 * An enumeration with supported rounding modes.
 */
typedef enum _FpuRMode
{
    FPU_RMODE_RN,            /** Round to Nearest mode */
    FPU_RMODE_RP,            /** Round towards Plus Infinity mode */
    FPU_RMODE_RM,            /** Round towards Minus Infinity mode */
    FPU_RMODE_RZ             /** Round towards Zero mode */
} FpuRMode;



void fpu_enable(void);

void fpu_disable(void);

void fpu_enableStacking(void);

void fpu_enableLazyStacking(void);

void fpu_disableStacking(void);

void fpu_setHalfPrecisionMode(FpuHalfPrecisionMode mode);

void fpu_setNanMode(FpuNanMode mode);

void fpu_setFlushToZero(bool fz);

void fpu_setRoundingMode(FpuRMode mode);

#endif  /* _FPU_H_ */

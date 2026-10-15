/*
 * SystemPara.h
 *
 *  Created on: 2026. 9. 15.
 *      Author: h04
 */

#ifndef CORE_SERVICE_SYSTEMPARA_H_
#define CORE_SERVICE_SYSTEMPARA_H_
/*************************************************************************
 * INCLUDES
 *************************************************************************/
#include <stdint.h>
#include "hw_types.h"

#include "Fmath.h"
#include "math.h"

/*************************************************************************
 * GLOBAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
 *************************************************************************/

/*************************************************************************
 * GLOBAL CONSTANT MACROS
 *************************************************************************/
#define TSAMPLE_1U   ((float32_t)0.000001f)/*1usec*/
#define TSAMPLE_10U  ((float32_t)0.00001f)/*10usec*/
#define TSAMPLE_16U  ((float32_t)0.0000167f)/*16usec*/
#define TSAMPLE_20U  ((float32_t)0.00002f)/*20usec*/
#define TSAMPLE_25U  ((float32_t)0.000025f)/*25usec*/
#define TSAMPLE_33U  ((float32_t)0.0000333f)/*33.3usec*/
#define TSAMPLE_30U  ((float32_t)0.00003f)/*30usec*/
#define TSAMPLE_40U  ((float32_t)0.00004f)/*40usec*/
#define TSAMPLE_80U  ((float32_t)0.00008f) //////////////////////0927
#define TSAMPLE_100U ((float32_t)0.0001f)/*100usec*/
#define TSAMPLE_150U ((float32_t)0.00015f)/*150usec*/
#define TSAMPLE_400U ((float32_t)0.0004f)/*400usec*/
#define TSAMPLE_500U ((float32_t)0.0005f)/*500usec*/
#define TSAMPLE_600U ((float32_t)0.0006f)/*600usec*/
#define TSAMPLE_1M ((float32_t)0.001f) /*1msec*/

#define FSAMPLE_10K ((float32_t)10000.0f)/*10kHz*/

#define DEFINE_TSAMPLE  TSAMPLE_100U
#define INV_DEFINE_TSAMPLE  (1.0f/DEFINE_TSAMPLE)

#define CUTOFF_FREQ_5HZ             ((float32_t)5.0f)
#define CUTOFF_FREQ_10HZ            ((float32_t)10.0f)
#define CUTOFF_FREQ_20HZ            ((float32_t)20.0f)
#define CUTOFF_FREQ_100HZ           ((float32_t)100.0f)
#define CUTOFF_FREQ_1000HZ          ((float32_t)1000.0f)
#define CUTOFF_FREQ_500HZ           ((float32_t)500.0f)
#define CUTOFF_FREQ_1KHZ            ((float32_t)1000.0f)
#define CUTOFF_FREQ_1500HZ          ((float32_t)1500.0f)
#define CUTOFF_FREQ_3KHZ            ((float32_t)3000.0f)
#define CUTOFF_FREQ_6KHZ            ((float32_t)6000.0f)
#define CUTOFF_FREQ_10KHZ           ((float32_t)10000.0f)
#define CUTOFF_FREQ_50KHZ           ((float32_t)50000.0f)
#define CUTOFF_FREQ_100KHZ          ((float32_t)100000.0f)

#define BSET 1
#define BCLR 0
#endif /* CORE_SERVICE_SYSTEMPARA_H_ */

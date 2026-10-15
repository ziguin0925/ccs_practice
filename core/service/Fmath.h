/*
 * Fmath.h
 *
 *  Created on: 2026. 9. 15.
 *      Author: h04
 */

#ifndef CORE_SERVICE_FMATH_H_
#define CORE_SERVICE_FMATH_H_


/*************************************************************************
 * INCLUDES
 *************************************************************************/
#define ECLIPSE

#ifndef ECLIPSE

#else
#include "math.h"
#endif
/*************************************************************************
 * GLOBAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
 *************************************************************************/
/*************************************************************************
 * GLOBAL CONSTANT MACROS
 *************************************************************************/



/*Inverse Factorials*/
#define F2      ((float32_t)(1.0f/2.0f))
#define F3      ((float32_t)(F2/3.0f))
#define F4      ((float32_t)(F3/4.0f))
#define F5      ((float32_t)(F4/5.0f))
#define F6      ((float32_t)(F5/6.0f))
#define F7      ((float32_t)(F6/7.0f))
#define F8      ((float32_t)(F7/8.0f))
#define F9      ((float32_t)(F8/9.0f))
#define F10     ((float32_t)(F9/10.0f))
#define F11     ((float32_t)(F10/11.0f))
#define F12     ((float32_t)(F11/12.0f))
#define F13     ((float32_t)(F12/13.0f))
#define F14     ((float32_t)(F13/14.0f))
#define F15     ((float32_t)(F14/15.0f))

/*Useful floating-point Constants*/
#define PI          ((float32_t)3.1415926536f)  /*PI*/
#define PI2         ((float32_t)6.2831853072f)  /*2*PI*/
#define INV_DPI     ((float32_t)0.1591549431f) /*1/(2*PI)*/
#define PI2_INV3    ((float32_t)2.0943951024f)  /*2*PI/3*/
#define INV_PI      ((float32_t)0.3183098862f)  /*1/PI*/
#define SQRT2       ((float32_t)1.4142135624f)  /*sqrt(2)*/
#define INV_SQRT2   ((float32_t)0.7071067812f)  /*1/sqrt(2)*/
#define SQRT3       ((float32_t)1.7320508075f)  /*sqrt(3)*/
#define INV_SQRT3   ((float32_t)0.5773502691f)  /*1/sqrt(3)*/
#define FLOAT_HALF  ((float32_t)0.5f)           /*1/2*/

#define INV_PI2         ((float32_t)(1.0f/6.2831853072f))   /*2*PI*/
#define HALF_PI     (PI/2)

#define INT_ONE     (1)
#define INT_TWO     (2)
#define INT_ZERO    (0)
#define FLOAT_ZERO  ((float32_t)0.0f)
#define FLOAT_ONE   ((float32_t)1.0f)
#define FLOAT_TWO   ((float32_t)2.0f)

#define FREQ        (60)


/*************************************************************************
 * GLOBAL FUNCTION PROTOTYPES
 *************************************************************************/
#ifndef ECLIPSE
/*
#define SquareRoot(fResult, fX)                             \
{                                                           \
    uint32_t ulIdx = 0;                                     \
    fResult = 1.0f;                                         \
    for(ulIdx = 0; ulIdx < 50; ulIdx++)                     \
    {                                                       \
        fResult = 0.5f * ( fResult + (fX / fResult) );      \
    }                                                       \
}
*/
#else
#define SquareRoot(fResult, fX)                             \
{                                                           \
    fResult = sqrtf(fX);                                    \
}
#endif
/*********************************************
 * \brief sine calculation by Taylor series
 *
 * \param angle(radian), angle(radian)^2
 *
 * \return sine(angle(radian)) output
 *
 * \remarks
 *
 ***********************************************/
#define TaylorSine(fResult, fX, fX2)    (fResult = ((fX)*((float32_t)1.-((fX2)*(F3-((fX2)*(F5-((fX2)*(F7-((fX2)*(F9-((fX2)*(F11-((fX2)*(F13-((fX2)*F15))))))))))))))))


/*********************************************
 * \brief sine calculation by Taylor series
 *
 * \param angle(radian)^2
 *
 * \return cosine(angle(radian)) output
 *
 * \remarks
 *
 **********************************************/
#define TaylorCosine(fResult, fX2)  (fResult = ((float32_t)1.-((fX2)*(F2-((fX2)*(F4-((fX2)*(F6-((fX2)*(F8-((fX2)*(F10-((fX2)*(F12-((fX2)*F14)))))))))))))))


/*********************************************
 * \brief bound the range of an input between -PI to PI
 *
 * \param angle (radian)
 *
 * \return -PI <= angle (radian) <= PI
 *
 * \remarks
 *
 **********************************************/


#define Bound_PI(fX)    \
{                       \
    while(fX > PI)      \
    {                   \
        fX -= PI2;      \
    }                   \
    while(fX < -PI)     \
    {                   \
        fX += PI2;      \
    }                   \
}

/*********************************************
 * \brief bound the range of an input between Min to Max
 *
 * \param input, minimum value, maximum value
 *
 * \return BSET/BCLR
 *
 * \remarks
 *
 **********************************************/

#define Bound_In(ucResult, fX, fmin, fmax)  \
{                                           \
    float32_t fTemp = fX;                   \
    if(fTemp > fmin && fTemp < fmax)        \
    {                                       \
        ucResult = BSET;                    \
    }                                       \
    else                                    \
    {                                       \
        ucResult = BCLR;                    \
    }                                       \
}

/*********************************************
 * \brief Min select
 *
 * \param inputA, inputB
 *
 * \return min value
 *
 * \remarks
 *
 **********************************************/

#define Min_Select(fResult, fA,  fB)\
{                                   \
    if((fA) <= (fB))                \
    {                               \
        (fResult) = (fA);           \
    }                               \
    else                            \
    {                               \
        (fResult) = (fB);           \
    }                               \
}


#define Max_Select(fResult, fA,  fB)\
{                                   \
    if((fA) <= (fB))                \
    {                               \
        (fResult) = (fB);           \
    }                               \
    else                            \
    {                               \
        (fResult) = (fA);           \
    }                               \
}
/*********************************************
 * \brief Limit the range of an input between Min to Max
 *
 * \param input, minimum value, maximum value
 *
 * \return BSET/BCLR
 *
 * \remarks
 *
 **********************************************/

#define Limiter(Val, fmin, fmax)\
{                               \
    if((Val) <= (fmin))         \
    {                           \
        (Val) = (fmin);         \
    }                           \
    else if((Val) > (fmax))     \
    {                           \
        (Val) = (fmax);         \
    }                           \
}


#endif /* CORE_SERVICE_FMATH_H_ */

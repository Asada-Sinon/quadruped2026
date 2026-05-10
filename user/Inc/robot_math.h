#ifndef ROBOT_MATH_H
#define ROBOT_MATH_H

#include <stdint.h>

static inline float Robot_AbsF(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static inline float Robot_ClampF(float x, float min_val, float max_val)
{
    if (x < min_val)
    {
        return min_val;
    }
    if (x > max_val)
    {
        return max_val;
    }
    return x;
}

static inline uint8_t Robot_IsFiniteReasonableF(float x)
{
    return ((x == x) && (x <= 1000000.0f) && (x >= -1000000.0f)) ? 1U : 0U;
}

#endif /* ROBOT_MATH_H */

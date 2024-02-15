#include <cmath>
#include "include/lib.h"

float lib::euclideanDistance2dAsFloat(float x1, float y1, float x2, float y2)
{
    float xDiff = x2 - x1;
    float yDiff = y2 - y1;
    return std::sqrt(xDiff * xDiff + yDiff * yDiff);
}

float lib::arctan(float x)
{
    // IMPLEMENTATION NEEDED
    return std::atan(x);
}

int lib::signum(float x)
{
    // WANT BETTER IMPLEMENTATION
    return x > 0 ? 1 : (x == 0 ? 0 : -1);
}

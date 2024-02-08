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
    // TODO
}

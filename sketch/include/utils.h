#ifndef UTILS
#define UTILS

#include "structs.h"

class utils
{
public:
    // At startup, get which round you are meant to be
    // running the program in.
    static Round getRound(Robot& robot);
    // If you are still trying to leave your charging station,
    // this function will tell you where the wall is so you
    // can drive away from it.
    static Direction getDirectionOfNearestWall(ChargingStation cs);
};

#endif

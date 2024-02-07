#ifndef UTILS
#define UTILS

#include "structs.h"

class utils
{
public:
    // At startup, get which round you are meant to be
    // running the program in.
    static Round getRound(Robot& robot);
};

#endif

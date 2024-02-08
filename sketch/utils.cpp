#include "include/utils.h"

Round utils::getRound(Robot& robot)
{
    // TODO
    return Round::NONE;
}

Direction utils::getDirectionOfNearestWall(ChargingStation cs)
{
    switch (cs)
    {
        case ChargingStation::A:
        case ChargingStation::C:
        case ChargingStation::E:
        case ChargingStation::G:
            return Direction::RIGHT;
        case ChargingStation::B:
        case ChargingStation::D:
        case ChargingStation::F:
        case ChargingStation::H:
            return Direction::LEFT;
    }
}

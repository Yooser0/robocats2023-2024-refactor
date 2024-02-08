#include <array>
#include "include/utils.h"
#include "include/structs.h"
#include "include/consts.h"
#include "include/lib.h"

// At startup, get which round you are meant to be
// running the program in.
Round utils::getRound(Robot& robot)
{
    // TODO
    return Round::NONE;
}

// If you are still trying to leave your charging station,
// this function will tell you where the wall is so you
// can drive away from it.
Direction utils::getDirectionOfNearestWall(ChargingStation station)
{
    switch (station)
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

// Finds the local (relative to the robot) angle of movement
// that would get the robot away from the given obstacles.
float utils::getAngleInDegreesAwayFromObstacles(
    const std::array<PositionInCentimeters, NUM_OBSTACLE_DETECTORS>& obstaclesRelativeToRobot
)
{
    // TODO
    return 0.0;
}

// Returns the global position associated with the given
// charging station.
PositionInCentimeters utils::chargingStationPosition(ChargingStation station)
{
    switch (station)
    {
        case ChargingStation::A: return STATION_A_POSITION;
        case ChargingStation::B: return STATION_B_POSITION;
        case ChargingStation::C: return STATION_C_POSITION;
        case ChargingStation::D: return STATION_D_POSITION;
        case ChargingStation::E: return STATION_E_POSITION;
        case ChargingStation::F: return STATION_F_POSITION;
        case ChargingStation::G: return STATION_G_POSITION;
        case ChargingStation::H: return STATION_H_POSITION;
    }
}

// Computes if you are in range of the given position to
// qualify as being there.
bool utils::isAtPosition(PositionInCentimeters robotPosition, PositionInCentimeters checkPosition, float checkRadius)
{
    return lib::euclideanDistance2dAsFloat(robotPosition.x, robotPosition.y, checkPosition.x, checkPosition.y) <= checkRadius;
}

float utils::distanceBetweenPositions(PositionInCentimeters pos1, PositionInCentimeters pos2)
{
    return lib::euclideanDistance2dAsFloat(pos1.x, pos1.y, pos2.x, pos2.y);
}

// Computes the angle of the given position relative to the
// given origin position.
float utils::angleOfPositionRadians(PositionInCentimeters position, PositionInCentimeters origin)
{
    return lib::arctan((position.y - origin.y) / (position.x - origin.x));
}

#ifndef UTILS
#define UTILS

#include <array>
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
    static Direction getDirectionOfNearestWall(ChargingStation station);
    // Finds the local (relative to the robot) angle of movement
    // that would get the robot away from the given obstacles.
    static float getAngleInDegreesAwayFromObstacles(
        const std::array<PositionInCentimeters, NUM_OBSTACLE_DETECTORS>& obstaclesRelativeToRobot
    );
    // Returns the global position associated with the given
    // charging station.
    static PositionInCentimeters chargingStationPosition(ChargingStation station);
    // Computes if you are in range of the given position to
    // qualify as being there.
    static bool isAtPosition(PositionInCentimeters robotPosition, PositionInCentimeters checkPosition, float checkRadius);
    static float distanceBetweenPositions(PositionInCentimeters pos1, PositionInCentimeters pos2);
    // Computes the angle of the given position relative to the
    // given origin position.
    static float angleOfPositionRadians(PositionInCentimeters position, PositionInCentimeters origin);
};

#endif

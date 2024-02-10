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
    // Returns the predetermined angle of the perpendicular wall
    // of the given charging station.
    static float angleInDegreesTowardsPerpendicularWall(ChargingStation station);
    // You must use this function when finding the difference
    // between one angle in radians and another.
    static float getAngleDiffInDegrees(float lhsAngle, float rhsAngle);
    // You must use this function when finding the difference
    // between on angle in radians and another.
    static float getAngleDiffInRadians(float lhsAngle, float rhsAngle);
    // Computes if the robot is in line or past the given charging
    // station after moving backward from the perpendicular wall or
    // if it still needs to keep moving backward.
    static bool isInLineWithChargerOrGreater(ChargingStation station, PositionInCentimeters position);
    // Returns which side of its corner the given charging station
    // is on.
    static Direction whichSideOfCorner(ChargingStation station);
};

#endif

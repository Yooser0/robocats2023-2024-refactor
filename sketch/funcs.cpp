#include <Arduino.h>
#include <limits>
#include <cmath>
#include "include/funcs.h"
#include "include/structs.h"
#include "include/consts.h"
#include "include/utils.h"

Funcs::Funcs(Robot& robot) : robot(robot) {}

// Seeding round implementation

SeedingRoundState Funcs::turningTowardsStartCornerFromStart()
{
    robot.turnLeft(90, SEEDING_ROUND_TURN_SLOWLY_ANGULAR_VELOCITY);
    return SeedingRoundState::MOVING_TOWARDS_START_CORNER_FROM_START;
}

SeedingRoundState Funcs::movingTowardsStartCornerFromStart()
{
    robot.moveForward(LEFT_START_CORNER.x - START.x, CORNER_TO_CENTER_VELOCITY);
    return SeedingRoundState::TURNING_TOWARDS_BUTTON_CORNER_FROM_START_CORNER;
}

SeedingRoundState Funcs::turningTowardsButtonCornerFromStartCorner()
{
    robot.turnRight(90, SEEDING_ROUND_TURN_SLOWLY_ANGULAR_VELOCITY);
    return SeedingRoundState::MOVING_TOWARDS_BUTTON_CORNER_FROM_START_CORNER;
}

SeedingRoundState Funcs::movingTowardsButtonCornerFromStartCorner()
{
    robot.moveForward(LEFT_BUTTON_CORNER.y - LEFT_START_CORNER.y, MOVING_TOWARDS_BUTTON_CORNER_FROM_START_CORNER_VELOCITY);
    return SeedingRoundState::TURNING_TOWARDS_CENTER_FROM_BUTTON_CORNER;
}

SeedingRoundState Funcs::turningTowardsCenterFromButtonCorner()
{
    robot.turnRight(90, SEEDING_ROUND_TURN_SLOWLY_ANGULAR_VELOCITY);
    return SeedingRoundState::MOVING_TOWARDS_CENTER_FROM_BUTTON_CORNER;
}

SeedingRoundState Funcs::movingTowardsCenterFromButtonCorner()
{
    robot.moveForward(CENTER_NEAR_BUTTON.x - LEFT_BUTTON_CORNER.x, CORNER_TO_CENTER_VELOCITY);
    return SeedingRoundState::TURNING_TOWARDS_BUTTON_FROM_CENTER;
}

SeedingRoundState Funcs::turningTowardsButtonFromCenter()
{
    robot.turnLeft(90, SEEDING_ROUND_TURN_SLOWLY_ANGULAR_VELOCITY);
    return SeedingRoundState::MOVING_TOWARDS_BUTTON_FROM_CENTER;
}

SeedingRoundState Funcs::movingTowardsButtonFromCenter()
{
    robot.forward(MOVING_TOWARDS_BUTTON_FROM_CENTER_VELOCITY);
    
    TimeDerivativeInSeconds derivativeOfAngularVelocity(robot.getAngularVelocityRef());
    TimeIntegralInSeconds integralOfDistanceFromCenter(robot.getXPositionRef(), -CENTER_NEAR_BUTTON.x);
    bool hitTheWall = false;
    while (!hitTheWall)
    {
        robot.changeAngularVelocity(
            -(CENTER_NEAR_BUTTON.x - robot.getXPosition()) * MOVING_TOWARDS_BUTTON_FROM_CENTER_PROPORTIONAL_PARAMETER +
            -derivativeOfAngularVelocity.getDerivative() * MOVING_TOWARDS_BUTTON_FROM_CENTER_DERIVATIVE_PARAMETER +
            -integralOfDistanceFromCenter.getIntegral() * MOVING_TOWARDS_BUTTON_FROM_CENTER_INTEGRAL_PARAMETER
        );
        integralOfDistanceFromCenter.updateIntegral();

        hitTheWall = robot.isTouchingWall();
    }
    robot.brake();

    return SeedingRoundState::MOVING_TOWARDS_CENTER_FROM_BUTTON;
}

SeedingRoundState Funcs::movingTowardsCenterFromButton()
{
    robot.moveBackward((BUTTON.y - FRONT_OF_ROBOT.y) - CENTER_NEAR_BUTTON.y, MOVING_TOWARDS_CENTER_FROM_BUTTON_VELOCITY);
    return SeedingRoundState::TURNING_TOWARDS_BUTTON_CORNER_FROM_CENTER;
}

SeedingRoundState Funcs::turningTowardsButtonCornerFromCenter()
{
    robot.turnLeft(90, SEEDING_ROUND_TURN_SLOWLY_ANGULAR_VELOCITY);
    return SeedingRoundState::MOVING_TOWARDS_BUTTON_CORNER_FROM_CENTER;
}

SeedingRoundState Funcs::movingTowardsButtonCornerFromCenter()
{
    robot.moveForward(CENTER_NEAR_BUTTON.x - LEFT_BUTTON_CORNER.x, MOVING_TOWARDS_BUTTON_CORNER_FROM_CENTER_VELOCITY);
    return SeedingRoundState::TURNING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER;
}

SeedingRoundState Funcs::turningTowardsStartCornerFromButtonCorner()
{
    robot.turnLeft(90, SEEDING_ROUND_TURN_SLOWLY_ANGULAR_VELOCITY);
    return SeedingRoundState::MOVING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER;
}

SeedingRoundState Funcs::movingTowardsStartCornerFromButtonCorner()
{
    robot.moveForward(LEFT_BUTTON_CORNER.y - LEFT_START_CORNER.y, MOVING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER_VELOCITY);
    return SeedingRoundState::STOPPED;
}

// Elimination round implementation

EliminationRoundState Funcs::powerOn()
{
    delay((int)(POWER_ON_WAIT_TIME * UNIT_TO_MILLI));
    return EliminationRoundState::MOVING_BACKWARDS_FROM_STATION;
}

EliminationRoundState Funcs::movingBackwardsFromStation()
{
    robot.backward(MOVING_BACKWARDS_FROM_STATION_VELOCITY);

    bool coastIsClear = true;
    bool farEnoughFromWall = false;
    while (coastIsClear && !farEnoughFromWall)
    {
        if (robot.getObstacleDistanceBehind() < MOVING_BACKWARDS_FROM_STATION_OBSTACLE_BEHIND_MIN_DISTANCE)
            coastIsClear = false;
        if (robot.getObstacleDistanceAhead() > MOVING_BACKWARDS_FROM_STATION_OBSTACLE_AHEAD_MAX_DISTANCE)
            farEnoughFromWall = true;
    }
    robot.brake();

    return EliminationRoundState::TURNING_AWAY_FROM_WALL;
}

EliminationRoundState Funcs::turningAwayFromWall()
{
    robot.forwardCircle(
        -(int)utils::getDirectionOfNearestWall(robot.getPreviousStation()) * TURNING_AWAY_FROM_WALL_ANGULAR_VELOCITY,
        TURNING_AWAY_FROM_WALL_TURN_RADIUS
    );

    float initialAngle = robot.getAngleDegrees();
    bool turnedEnough = false;
    while (!turnedEnough)
    {
        if (robot.isMovingObjectApproaching())
        {
            robot.brake();
            return EliminationRoundState::MOVING_AWAY_FROM_OBSTACLES;
        }

        float angleDiff = abs(robot.getAngleDegrees() - initialAngle);
        turnedEnough = angleDiff >= 90;
    }
    robot.brake();

    return EliminationRoundState::MOVING_TOWARDS_NEXT_STATION;
}

EliminationRoundState Funcs::movingAwayFromObstacles()
{
    while (robot.isObstacleInRange(MOVING_AWAY_FROM_OBSTACLES_MAX_DISTANCE))
        robot.go(utils::getAngleInDegreesAwayFromObstacles(robot.getObstacles()), MOVING_AWAY_FROM_OBSTACLES_VELOCITY);

    return EliminationRoundState::MOVING_TOWARDS_NEXT_STATION;
}

float movingTowardsNextStationRateOfTravel(float distanceFromStation);
EliminationRoundState Funcs::movingTowardsNextStation()
{
    PositionInCentimeters nextStationPosition = utils::chargingStationPosition(robot.getNextStation());

    TimeDerivativeInSeconds derivativeOfAngularVelocity(robot.getAngularVelocityRef());
    bool isAtNextStation = false;
    while (!isAtNextStation)
    {
        if (robot.isMovingObjectApproaching())
        {
            robot.brake();
            return EliminationRoundState::MOVING_AWAY_FROM_OBSTACLES;
        }

        robot.forward(movingTowardsNextStationRateOfTravel(
            utils::distanceBetweenPositions(robot.getPosition(), nextStationPosition)
        ));

        float nextStationAngleDiff = utils::getAngleDiffInRadians(
            utils::angleOfPositionRadians(nextStationPosition, robot.getPosition()), robot.getAngleRadians()
        );
        robot.changeAngularVelocity(
            nextStationAngleDiff * MOVING_TOWARDS_NEXT_STATION_PROPORTIONAL_PARAMETER +
            -derivativeOfAngularVelocity.getDerivative() * MOVING_TOWARDS_NEXT_STATION_DERIVATIVE_PARAMETER
        );

        isAtNextStation = utils::isAtPosition(
            robot.getPosition(),
            nextStationPosition,
            MOVING_TOWARDS_NEXT_STATION_AT_STATION_RANGE_RADIUS
        );
    }
    return EliminationRoundState::TURNING_TOWARDS_PERPENDICULAR_BUTTON_WALL;
}

float movingTowardsNextStationRateOfTravel(float distanceFromStation)
{
    return MOVING_TOWARDS_NEXT_STATION_MAX_VELOCITY *
        std::tanh(distanceFromStation * MOVING_TOWARDS_NEXT_STATION_MOVEMENT_PARAMETER);
}

EliminationRoundState Funcs::turningTowardsPerpendicularButtonWall()
{
    float wallAngle = utils::angleInDegreesTowardsPerpendicularWall(robot.getNextStation());
    float angleDiff = utils::getAngleDiffInDegrees(wallAngle, robot.getAngleDegrees());
    robot.turn(angleDiff, TURNING_TOWARDS_PERPENDICULAR_BUTTON_WALL_ANGULAR_VELOCITY);
    return EliminationRoundState::MOVING_TOWARDS_PERPENDICULAR_BUTTON_WALL;
}

EliminationRoundState Funcs::movingTowardsPerpendicularButtonWall()
{
    robot.forward(MOVING_TOWARDS_PERPENDICULAR_BUTTON_WALL_VELOCITY);
    while (!robot.isTouchingWall());

    delay((int)(MOVING_TOWARDS_PERPENDICULAR_BUTTON_WALL_HIT_WALL_DELAY * UNIT_TO_MILLI));
    robot.brake();

    return EliminationRoundState::MOVING_AWAY_FROM_PERPENDICULAR_BUTTON_WALL;
}

EliminationRoundState Funcs::movingAwayFromPerpendicularButtonWall()
{
    robot.backward(MOVING_AWAY_FROM_PERPENDICULAR_BUTTON_WALL_VELOCITY);
    while (!utils::isInLineWithChargerOrGreater(robot.getNextStation(), robot.getPosition()));
    robot.brake();

    return EliminationRoundState::TURNING_TOWARDS_CHARGER_FROM_CORNER;
}

EliminationRoundState Funcs::turningTowardsChargerFromCorner()
{
    robot.turn(
        (int)utils::whichSideOfCorner(robot.getNextStation()) * 90,
        TURNING_TOWARDS_CHARGER_FROM_CORNER_ANGULAR_VELOCITY
    );
    return EliminationRoundState::MOVING_TOWARDS_CHARGER_FROM_CORNER;
}

EliminationRoundState Funcs::movingTowardsChargerFromCorner()
{
    robot.forward(MOVING_TOWARDS_CHARGER_FROM_CORNER_VELOCITY);
    while (!robot.isTouchingWall());
    delay((int)(MOVING_TOWARDS_CHARGER_FROM_CORNER_HIT_WALL_DELAY * UNIT_TO_MILLI));

    return EliminationRoundState::CHARGING;
}

EliminationRoundState Funcs::charging()
{
    delay((int)(CHARGING_DELAY * UNIT_TO_MILLI));
    return EliminationRoundState::STOPPED;
}

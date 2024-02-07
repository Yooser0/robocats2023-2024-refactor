#include <Arduino.h>
#include "include/funcs.h"
#include "include/structs.h"
#include "include/consts.h"

Funcs::Funcs(Robot& robot) : robot(robot) {}

// Seeding round implementation

SeedingRoundState Funcs::turningTowardsStartCornerFromStart() {
    robot.turnLeft(90, TURN_SLOWLY_ANGULAR_VELOCITY);
    return SeedingRoundState::MOVING_TOWARDS_START_CORNER_FROM_START;
}

SeedingRoundState Funcs::movingTowardsStartCornerFromStart() {
    robot.moveForward(LEFT_START_CORNER.x - START.x, CORNER_TO_CENTER_VELOCITY);
    return SeedingRoundState::TURNING_TOWARDS_BUTTON_CORNER_FROM_START;
}

SeedingRoundState Funcs::turningTowardsButtonCornerFromStart() {
    robot.turnRight(90, TURN_SLOWLY_ANGULAR_VELOCITY);
    return SeedingRoundState::MOVING_TOWARDS_BUTTON_CORNER_FROM_START_CORNER;
}

SeedingRoundState Funcs::movingTowardsButtonCornerFromStartCorner() {
    robot.moveForward(LEFT_BUTTON_CORNER.y - LEFT_START_CORNER.y, MOVING_TOWARDS_BUTTON_CORNER_FROM_START_CORNER_VELOCITY);
    return SeedingRoundState::TURNING_TOWARDS_CENTER_FROM_BUTTON_CORNER;
}

SeedingRoundState Funcs::turningTowardsCenterFromButtonCorner() {
    robot.turnRight(90, TURN_SLOWLY_ANGULAR_VELOCITY);
    return SeedingRoundState::MOVING_TOWARDS_CENTER_FROM_BUTTON_CORNER;
}

SeedingRoundState Funcs::movingTowardsCenterFromButtonCorner() {
    robot.moveForward(CENTER_NEAR_BUTTON.x - LEFT_BUTTON_CORNER.x, CORNER_TO_CENTER_VELOCITY);
    return SeedingRoundState::TURNING_TOWARDS_BUTTON_FROM_CENTER;
}

SeedingRoundState Funcs::turningTowardsButtonFromCenter() {
    robot.turnLeft(90, TURN_SLOWLY_ANGULAR_VELOCITY);
    return SeedingRoundState::MOVING_TOWARDS_BUTTON_FROM_CENTER;
}

SeedingRoundState Funcs::movingTowardsButtonFromCenter() {
    bool hitTheWall = false;
    robot.forward(MOVING_TOWARDS_BUTTON_FROM_CENTER_VELOCITY);

    TimeDerivativeInSeconds derivativeOfAngularVelocity(robot.getAngularVelocityRef());
    TimeIntegralInSeconds integralOfDistanceFromCenter(robot.getXPositionRef(), -CENTER_NEAR_BUTTON.x);
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

SeedingRoundState Funcs::movingTowardsCenterFromButton() {
    robot.moveBackward((BUTTON.y - FRONT_OF_ROBOT.y) - CENTER_NEAR_BUTTON.y, MOVING_TOWARDS_CENTER_FROM_BUTTON_VELOCITY);
    return SeedingRoundState::TURNING_TOWARDS_BUTTON_CORNER_FROM_CENTER;
}

SeedingRoundState Funcs::turningTowardsButtonCornerFromCenter() {
    robot.turnLeft(90, TURN_SLOWLY_ANGULAR_VELOCITY);
    return SeedingRoundState::MOVING_TOWARDS_BUTTON_CORNER_FROM_CENTER;
}

SeedingRoundState Funcs::movingTowardsButtonCornerFromCenter() {
    robot.moveForward(CENTER_NEAR_BUTTON.x - LEFT_BUTTON_CORNER.x, MOVING_TOWARDS_BUTTON_CORNER_FROM_CENTER_VELOCITY);
    return SeedingRoundState::TURNING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER;
}

SeedingRoundState Funcs::turningTowardsStartCornerFromButtonCorner() {
    robot.turnLeft(90, TURN_SLOWLY_ANGULAR_VELOCITY);
    return SeedingRoundState::MOVING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER;
}

SeedingRoundState Funcs::movingTowardsStartCornerFromButtonCorner() {
    robot.moveForward(LEFT_BUTTON_CORNER.y - LEFT_START_CORNER.y, MOVING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER_VELOCITY);
    return SeedingRoundState::STOPPED;
}

// Elimination round implementation

EliminationRoundState Funcs::powerOn() {
    delay((int)(POWER_ON_WAIT_TIME * SECONDS_TO_MICROSECONDS));
    return EliminationRoundState::MOVING_BACKWARDS_FROM_STATION;
}

EliminationRoundState Funcs::movingBackwardsFromStation() {
    bool coastIsClear = true;
    bool farEnoughFromWall = false;
    while (coastIsClear && !farEnoughFromWall)
    {
        robot.backward(MOVING_BACKWARDS_FROM_STATION_VELOCITY);
        if (robot.getObstacleDistanceBehind() < MOVING_BACKWARDS_FROM_STATION_OBSTACLE_BEHIND_MIN_DISTANCE)
            coastIsClear = false;
        if (robot.getObstacleDistanceAhead() > MOVING_BACKWARDS_FROM_STATION_OBSTACLE_AHEAD_MAX_DISTANCE)
            farEnoughFromWall = true;
    }
    robot.brake();

    return EliminationRoundState::TURNING_AWAY_FROM_WALL;
}

EliminationRoundState Funcs::turningAwayFromWall() {
    float initialAngle = robot.getAngle();
    while (abs(robot.getAngle() - initialAngle) < 90 * DEGREES_TO_RADIANS)
    {
        // TODO
    }

    return EliminationRoundState::STOPPED;
}

EliminationRoundState Funcs::movingAwayFromObstacles() {
    // TODO
    return EliminationRoundState::STOPPED;
}

EliminationRoundState Funcs::movingTowardsNextStation() {
    // TODO
    return EliminationRoundState::STOPPED;
}

EliminationRoundState Funcs::turningTowardsPerpendicularButtonWall() {
    // TODO
    return EliminationRoundState::STOPPED;
}

EliminationRoundState Funcs::movingTowardsPerpendicularButtonWall() {
    // TODO
    return EliminationRoundState::STOPPED;
}

EliminationRoundState Funcs::movingAwayFromPerpendicularButtonWall() {
    // TODO
    return EliminationRoundState::STOPPED;
}

EliminationRoundState Funcs::turningTowardsChargerFromCorner() {
    // TODO
    return EliminationRoundState::STOPPED;
}

EliminationRoundState Funcs::movingTowardsChargerFromCorner() {
    // TODO
    return EliminationRoundState::STOPPED;
}

EliminationRoundState Funcs::charging() {
    // TODO
    return EliminationRoundState::STOPPED;
}

#include <array>
#include <Arduino.h>
#include "include/structs.h"
#include "include/consts.h"

// Position implementation

PositionInCentimeters::PositionInCentimeters() : x(0), y(0) {}

PositionInCentimeters::PositionInCentimeters(float x, float y) : x(x), y(y) {}

// Robot implementation

Robot::Robot() : positionInCentimeters(START)
{
    // TODO
}

void Robot::turnLeft(unsigned int angleInDegrees, float angularVelocityInRadians)
{
    // TODO
    return;
}

void Robot::turnRight(unsigned int angleInDegrees, float angularVelocityInRadians)
{
    // TODO
    return;
}

void Robot::turn(unsigned int angleInDegrees, float angularVelocityInRadians)
{
    // TODO
    return;
}

void Robot::moveForward(float moveDistanceInCentimeters, float velocityInCentimeters)
{
    // TODO
    return;
}

void Robot::moveBackward(float moveDistanceInCentimeters, float velocityInCentimeters)
{
    // TODO
    return;
}

void Robot::forward(float velocityInCentimeters)
{
    // TODO
    return;
}

void Robot::backward(float velocityInCentimeters)
{
    // TODO
    return;
}

void Robot::forwardCircle(float angularVelocityInRadians, float radiusInCentimeters)
{
    // TODO
    return;
}

void Robot::go(float directionInDegreesRelativeToRobot, float velocityInCentimeters)
{
    // TODO
    return;
}

void Robot::brake()
{
    // TODO
    return;
}

void Robot::changeAngularVelocity(float angularVelocityInRadians)
{
    // TODO
    return;
}

float Robot::getObstacleDistanceAhead()
{
    // TOOD
    return 0.0;
}

float Robot::getObstacleDistanceBehind()
{
    // TODO
    return 0.0;
}

bool Robot::isMovingObjectApproaching()
{
    // TODO
    return false;
}

bool Robot::isObstacleInRange(float radiusInCentimeters)
{
    // TODO
    return false;
}

bool Robot::isTouchingWall()
{
    // TODO
    return false;
}

PositionInCentimeters Robot::getPosition()
{
    return positionInCentimeters;
}

float Robot::getXPosition()
{
    return positionInCentimeters.x;
}

float& Robot::getXPositionRef()
{
    return positionInCentimeters.x;
}

std::array<PositionInCentimeters, NUM_OBSTACLE_DETECTORS>& Robot::getObstacles()
{
    return obstaclesRelativeToRobot;
}

ChargingStation Robot::getPreviousStation()
{
    return previousStation;
}

ChargingStation Robot::getNextStation()
{
    switch (previousStation)
    {
        case ChargingStation::A: return ChargingStation::D;
        case ChargingStation::D: return ChargingStation::H;
        case ChargingStation::H: return ChargingStation::F;
        case ChargingStation::F: return ChargingStation::B;
        case ChargingStation::B: return ChargingStation::G;
        case ChargingStation::G: return ChargingStation::E;
        case ChargingStation::E: return ChargingStation::C;
        case ChargingStation::C: return ChargingStation::A;
    }
}

float Robot::getAngleDegrees()
{
    return angleInRadians * RADIANS_TO_DEGREES;
}

float Robot::getAngleRadians()
{
    return angleInRadians;
}

float& Robot::getAngularVelocityRef()
{
    return angularVelocityInRadians;
}

// Derivative implementation

TimeDerivativeInSeconds::TimeDerivativeInSeconds(float& input) : input(input)
{
    prevInput = input;
    prevTimeInSeconds = micros() * MICRO_TO_UNIT;
}

float TimeDerivativeInSeconds::getDerivative()
{
    // TODO
    return 0.0;
}

// Integral implementation

TimeIntegralInSeconds::TimeIntegralInSeconds(float& input, float inputBias) : input(input), inputBias(inputBias)
{
    prevInput = input;
    prevTimeInSeconds = micros() * MICRO_TO_UNIT;
    integral = 0;
}

float TimeIntegralInSeconds::getIntegral()
{
    return integral;
}

void TimeIntegralInSeconds::updateIntegral()
{
    // TODO
}

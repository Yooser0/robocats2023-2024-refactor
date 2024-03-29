#ifndef STRUCTS
#define STRUCTS

#include <array>
#include <TB6612_ESP32.h>
#include "consts.h"

enum class Round
{
    SEEDING,
    ELIMINATION,
    NONE,
};

// Check `SeedingRound` diagram for what each state is accomplishing
enum class SeedingRoundState
{
    TURNING_TOWARDS_START_CORNER_FROM_START,
    MOVING_TOWARDS_START_CORNER_FROM_START,
    TURNING_TOWARDS_BUTTON_CORNER_FROM_START_CORNER,
    MOVING_TOWARDS_BUTTON_CORNER_FROM_START_CORNER,
    TURNING_TOWARDS_CENTER_FROM_BUTTON_CORNER,
    MOVING_TOWARDS_CENTER_FROM_BUTTON_CORNER,
    TURNING_TOWARDS_BUTTON_FROM_CENTER,
    MOVING_TOWARDS_BUTTON_FROM_CENTER,
    MOVING_TOWARDS_CENTER_FROM_BUTTON,
    TURNING_TOWARDS_BUTTON_CORNER_FROM_CENTER,
    MOVING_TOWARDS_BUTTON_CORNER_FROM_CENTER,
    TURNING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER,
    MOVING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER,
    STOPPED,
};

enum class EliminationRoundState
{
    POWER_ON,
    MOVING_BACKWARDS_FROM_STATION,
    TURNING_AWAY_FROM_WALL,
    MOVING_AWAY_FROM_OBSTACLES,
    MOVING_TOWARDS_NEXT_STATION,
    TURNING_TOWARDS_PERPENDICULAR_BUTTON_WALL,
    MOVING_TOWARDS_PERPENDICULAR_BUTTON_WALL,
    MOVING_AWAY_FROM_PERPENDICULAR_BUTTON_WALL,
    TURNING_TOWARDS_CHARGER_FROM_CORNER,
    MOVING_TOWARDS_CHARGER_FROM_CORNER,
    CHARGING,
    STOPPED,
};

class PositionInCentimeters
{
public:
    float x, y;

    PositionInCentimeters();
    PositionInCentimeters(float x, float y);
};

enum class ChargingStation
{
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
};

struct MotorInfo {
    int in1Pin;
    int in2Pin;
    int pwmPin;
    int offset;
    int stbyPin;
    int freq;
    int resolution;
    int channelPin;
};

class Robot
{
    Motor motorLeft;
    Motor motorRight;

    PositionInCentimeters positionInCentimeters;
    std::array<PositionInCentimeters, NUM_OBSTACLE_DETECTORS> obstaclesRelativeToRobot;
    ChargingStation previousStation;
    float angleInRadians;
    float angularVelocityInRadians;
public:
    Robot(const MotorInfo& motorLeft, const MotorInfo& motorRight);

    void turnLeft(unsigned int angleInDegrees, float angularVelocityInRadians);
    void turnRight(unsigned int angleInDegrees, float angularVelocityInRadians);
    void turn(float angleInDegrees, float angularVelocityInRadians);
    void moveForward(float moveDistanceInCentimeters, float velocityInCentimeters);
    void moveBackward(float moveDistanceInCentimeters, float velocityInCentimeters);
    void forward(float velocityInCentimeters);
    void backward(float velocityInCentimeters);
    void forwardCircle(float angularVelocityInRadians, float radiusInCentimeters);
    void go(float directionInDegreesRelativeToRobot, float velocityInCentimeters);
    void brake();
    void changeAngularVelocity(float angularVelocityInRadians);

    float getObstacleDistanceAhead();
    float getObstacleDistanceBehind();
    bool isMovingObjectApproaching();
    bool isObstacleInRange(float radiusInCentimeters);
    bool isTouchingWall();

    PositionInCentimeters getPosition();
    float getXPosition();
    float& getXPositionRef();
    std::array<PositionInCentimeters, NUM_OBSTACLE_DETECTORS>& getObstacles();
    ChargingStation getPreviousStation();
    ChargingStation getNextStation();
    float getAngleDegrees();
    float getAngleRadians();
    float& getAngularVelocityRef();
private:
    void circle(float angularVelocityInRadians, float radiusInCentimeters);
    void moveCircle(float angleInDegrees, float angularVelocityInRadians, float radiusInCentimeters);
};

class IRSensor
{
private:
    int sensorPin; // Pin number for the IR sensor
    float distance_prev; // Previous distance measurement
    float prev_time; // Time of previous measurement
    static const size_t ROLLING_AVERAGE_SIZE = 20; // Size of the rolling average window
    float rolling_average[ROLLING_AVERAGE_SIZE]; // Array for storing past distance measurements
    size_t arr_i; // Index for the current position in the rolling average array

public:
    IRSensor(int pin); // Constructor that sets the sensor pin
    void setup(); // Setup method for initializing the sensor
    float getDistance(); // Method to get the current distance measurement
    // Additional methods as necessary...
};

class TimeDerivativeInSeconds
{
    float& input;
    float prevInput;
    float prevTimeInSeconds;
public:
    TimeDerivativeInSeconds(float& input);
    float getDerivative();
};

class TimeIntegralInSeconds
{
    float& input;
    float inputBias;
    float prevInput;
    float prevTimeInSeconds;
    float integral;
public:
    TimeIntegralInSeconds(float& input, float inputBias);
    float getIntegral();
    void updateIntegral();
};

// Positive if in the positive direction of angular
// velocity, and negative if in the negative direction
// of angular velocity.
enum class Direction
{
    LEFT = 1,
    RIGHT = -1,
};

#endif

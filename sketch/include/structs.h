#ifndef STRUCTS
#define STRUCTS

enum class Round
{
    SEEDING,
    ELIMINATION,
    NONE,
};

enum class SeedingRoundState
{
    TURNING_TOWARDS_START_CORNER_FROM_START,
    MOVING_TOWARDS_START_CORNER_FROM_START,
    TURNING_TOWARDS_BUTTON_CORNER_FROM_START,
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

    PositionInCentimeters(float x, float y);
};

class Robot
{
    PositionInCentimeters positionInCentimeters;
    float angle;
    float angularVelocity;
public:
    Robot();

    void turnLeft(unsigned int angleInDegrees, float angularVelocity);
    void turnRight(unsigned int angleInDegrees, float angularVelocity);
    void moveForward(float moveDistanceInCentimeters, float velocityInCentimeters);
    void moveBackward(float moveDistanceInCentimeters, float velocityInCentimeters);
    void forward(float velocityInCentimeters);
    void backward(float velocityInCentimeters);
    void brake();
    void changeAngularVelocity(float angularVelocity);

    float getObstacleDistanceAhead();
    float getObstacleDistanceBehind();

    bool isTouchingWall();
    float getXPosition();
    float& getXPositionRef();
    float getAngle();
    float& getAngularVelocityRef();
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

#endif

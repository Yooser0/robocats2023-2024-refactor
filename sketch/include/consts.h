#ifndef CONSTS
#define CONSTS

#include "structs.h"

// Note: All position measurements are recorded in centimeters,
// all time measurements are recorded in seconds, all angular
// distance measurements are recorded in degrees, and all angular
// velocity measurements are recorded in radians.

// Seeding round constants

// Velocity constants
extern const float CORNER_TO_CENTER_VELOCITY;
extern const float MOVING_TOWARDS_BUTTON_CORNER_FROM_START_CORNER_VELOCITY;
extern const float MOVING_TOWARDS_BUTTON_FROM_CENTER_VELOCITY;
extern const float MOVING_TOWARDS_CENTER_FROM_BUTTON_VELOCITY;
extern const float MOVING_TOWARDS_BUTTON_CORNER_FROM_CENTER_VELOCITY;
extern const float MOVING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER_VELOCITY;

// Angular velocity constants
extern const float TURN_SLOWLY_ANGULAR_VELOCITY;

// PID constants
extern const float MOVING_TOWARDS_BUTTON_FROM_CENTER_PROPORTIONAL_PARAMETER;
extern const float MOVING_TOWARDS_BUTTON_FROM_CENTER_DERIVATIVE_PARAMETER;
extern const float MOVING_TOWARDS_BUTTON_FROM_CENTER_INTEGRAL_PARAMETER;

// Field position constants
extern const PositionInCentimeters START;
extern const PositionInCentimeters LEFT_START_CORNER;
extern const PositionInCentimeters LEFT_BUTTON_CORNER;
extern const PositionInCentimeters CENTER_NEAR_BUTTON;
extern const PositionInCentimeters BUTTON;

// Local position constants
extern const PositionInCentimeters FRONT_OF_ROBOT;

// Elimination round constants

// Time constants
extern const float POWER_ON_WAIT_TIME;

// Velocity constants
extern const float MOVING_BACKWARDS_FROM_STATION_VELOCITY;

// Distance constants
extern const float MOVING_BACKWARDS_FROM_STATION_OBSTACLE_BEHIND_MIN_DISTANCE;
extern const float MOVING_BACKWARDS_FROM_STATION_OBSTACLE_AHEAD_MAX_DISTANCE;
extern const float TURNING_AWAY_FROM_WALL_TURN_RADIUS;
extern const float MOVING_AWAY_FROM_OBSTACLES_MAX_DISTANCE;

// Angle constants
extern const float TURNING_AWAY_FROM_WALL_ANGLE_OFFSET;

// Angular velocity constants
extern const float TURNING_AWAY_FROM_WALL_ANGULAR_VELOCITY;

// Universal constants

// Conversion constants
extern const float MICRO_TO_UNIT;
extern const unsigned int SECONDS_TO_MICROSECONDS;
extern const float RADIANS_TO_DEGREES;

#endif

#include <cmath>
#include "include/consts.h"
#include "include/structs.h"

// Note: All position measurements are recorded in centimeters,
// all time measurements are recorded in seconds, all angular
// distance measurements are recorded in degrees, and all angular
// velocity measurements are recorded in radians.

// Seeding round constants

// Velocity constants
extern const float CORNER_TO_CENTER_VELOCITY = 0.0;
extern const float MOVING_TOWARDS_BUTTON_CORNER_FROM_START_CORNER_VELOCITY = 0.0;
extern const float MOVING_TOWARDS_BUTTON_FROM_CENTER_VELOCITY = 0.0;
extern const float MOVING_TOWARDS_CENTER_FROM_BUTTON_VELOCITY = 0.0;
extern const float MOVING_TOWARDS_BUTTON_CORNER_FROM_CENTER_VELOCITY = 0.0;
extern const float MOVING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER_VELOCITY = 0.0;

// Angular velocity constants
extern const float SEEDING_ROUND_TURN_SLOWLY_ANGULAR_VELOCITY = 0.0;

// PID constants
extern const float MOVING_TOWARDS_BUTTON_FROM_CENTER_PROPORTIONAL_PARAMETER = 0.0;
extern const float MOVING_TOWARDS_BUTTON_FROM_CENTER_DERIVATIVE_PARAMETER = 0.0;
extern const float MOVING_TOWARDS_BUTTON_FROM_CENTER_INTEGRAL_PARAMETER = 0.0;

// Field position constants
extern const PositionInCentimeters START(0.0, 0.0);
extern const PositionInCentimeters LEFT_START_CORNER(0.0, 0.0);
extern const PositionInCentimeters LEFT_BUTTON_CORNER(0.0, 0.0);
extern const PositionInCentimeters CENTER_NEAR_BUTTON(0.0, 0.0);
extern const PositionInCentimeters BUTTON(0.0, 0.0);

// Local position constants
extern const PositionInCentimeters FRONT_OF_ROBOT(0.0, 0.0);

// Elimination round constants

// Time constants
extern const float POWER_ON_WAIT_TIME = 10.0;
extern const float MOVING_TOWARDS_PERPENDICULAR_BUTTON_WALL_HIT_WALL_DELAY = 0.0;
extern const float MOVING_TOWARDS_CHARGER_FROM_CORNER_HIT_WALL_DELAY = 0.0;
extern const float CHARGING_DELAY = 10.0;

// Velocity constants
extern const float MOVING_BACKWARDS_FROM_STATION_VELOCITY = 0.0;
extern const float MOVING_AWAY_FROM_OBSTACLES_VELOCITY = 0.0;
extern const float MOVING_TOWARDS_NEXT_STATION_MAX_VELOCITY = 0.0;
extern const float MOVING_TOWARDS_PERPENDICULAR_BUTTON_WALL_VELOCITY = 0.0;
extern const float MOVING_AWAY_FROM_PERPENDICULAR_BUTTON_WALL_VELOCITY = 0.0;
extern const float MOVING_TOWARDS_CHARGER_FROM_CORNER_VELOCITY = 0.0;

// Distance constants
extern const float MOVING_BACKWARDS_FROM_STATION_OBSTACLE_BEHIND_MIN_DISTANCE = 0.0;
extern const float MOVING_BACKWARDS_FROM_STATION_OBSTACLE_AHEAD_MAX_DISTANCE = INFINITY;
extern const float TURNING_AWAY_FROM_WALL_TURN_RADIUS = 0.0;
extern const float MOVING_AWAY_FROM_OBSTACLES_MAX_DISTANCE = 0.0;
extern const float MOVING_TOWARDS_NEXT_STATION_AT_STATION_RANGE_RADIUS = 0.0;

// Angle constants
extern const float TURNING_AWAY_FROM_WALL_ANGLE_OFFSET = 0.0;
extern const float ELIMINATION_ROUND_STARTING_ANGLE = 0.0;

// Angular velocity constants
extern const float TURNING_AWAY_FROM_WALL_ANGULAR_VELOCITY = 0.0;
extern const float TURNING_TOWARDS_CHARGER_FROM_CORNER_ANGULAR_VELOCITY = 0.0;

// Position constants
extern const PositionInCentimeters STATION_A_POSITION(0.0, 0.0);
extern const PositionInCentimeters STATION_B_POSITION(0.0, 0.0);
extern const PositionInCentimeters STATION_C_POSITION(0.0, 0.0);
extern const PositionInCentimeters STATION_D_POSITION(0.0, 0.0);
extern const PositionInCentimeters STATION_E_POSITION(0.0, 0.0);
extern const PositionInCentimeters STATION_F_POSITION(0.0, 0.0);
extern const PositionInCentimeters STATION_G_POSITION(0.0, 0.0);
extern const PositionInCentimeters STATION_H_POSITION(0.0, 0.0);

// PID / complex movement constants
extern const float MOVING_TOWARDS_NEXT_STATION_MOVEMENT_PARAMETER = 0.0;
extern const float MOVING_TOWARDS_NEXT_STATION_PROPORTIONAL_PARAMETER = 0.0;
extern const float MOVING_TOWARDS_NEXT_STATION_DERIVATIVE_PARAMETER = 0.0;

// Angular velocity constants
extern const float TURNING_TOWARDS_PERPENDICULAR_BUTTON_WALL_ANGULAR_VELOCITY = 0.0;

// Universal constants

// Conversion constants
extern const float MICRO_TO_UNIT = 0.000001;
extern const unsigned int UNIT_TO_MICRO = 1000000;
extern const unsigned int UNIT_TO_MILLI = 1000;
extern const float RADIANS_TO_DEGREES = 180 / M_PI;

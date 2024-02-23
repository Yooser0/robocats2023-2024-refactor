#include "include/utils.h"
#include "include/structs.h"
#include "include/funcs.h"
#include "include/consts.h"

MotorInfo motorLeft = { MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_LEFT_PWM, STBY, FREQUENCY, RESOLUTION, MOTOR_LEFT_CHANNEL };
MotorInfo motorRight = { MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_RIGHT_PWM, STBY, FREQUENCY, RESOLUTION, MOTOR_RIGHT_CHANNEL };
Robot robot(motorLeft, motorRight);
Funcs funcs = Funcs(robot);
Round whatRound;
SeedingRoundState seedingState;
EliminationRoundState elimState;
IRSensor irSensor(34); // Assuming pin 34 for the IR sensor

void setup()
{
    Serial.begin(115200); // Initialize serial communication at 115200 baud rate
    irSensor.setup(); // Initialize the IR sensor
    whatRound = utils::getRound(robot);

    switch (whatRound) {
        case Round::SEEDING:
            seedingState = SeedingRoundState::TURNING_TOWARDS_START_CORNER_FROM_START;
            break;
        case Round::ELIMINATION:
            elimState = EliminationRoundState::POWER_ON;
    }
}

void loop()
{
    float distance = irSensor.getDistance(); // Get the current distance
    Serial.print("Distance: ");
    Serial.println(distance, 2); // Print the distance with two decimal places

    delay(100); // Wait for a second before the next read
    switch (whatRound)
    {
    case Round::SEEDING:
        switch (seedingState)
        {
        case SeedingRoundState::TURNING_TOWARDS_START_CORNER_FROM_START:
            seedingState = funcs.turningTowardsStartCornerFromStart();
            break;
        case SeedingRoundState::MOVING_TOWARDS_START_CORNER_FROM_START:
            seedingState = funcs.movingTowardsStartCornerFromStart();
            break;
        case SeedingRoundState::TURNING_TOWARDS_BUTTON_CORNER_FROM_START_CORNER:
            seedingState = funcs.turningTowardsButtonCornerFromStartCorner();
            break;
        case SeedingRoundState::MOVING_TOWARDS_BUTTON_CORNER_FROM_START_CORNER:
            seedingState = funcs.movingTowardsButtonCornerFromStartCorner();
            break;
        case SeedingRoundState::TURNING_TOWARDS_CENTER_FROM_BUTTON_CORNER:
            seedingState = funcs.turningTowardsCenterFromButtonCorner();
            break;
        case SeedingRoundState::MOVING_TOWARDS_CENTER_FROM_BUTTON_CORNER:
            seedingState = funcs.movingTowardsCenterFromButtonCorner();
            break;
        case SeedingRoundState::TURNING_TOWARDS_BUTTON_FROM_CENTER:
            seedingState = funcs.turningTowardsButtonFromCenter();
            break;
        case SeedingRoundState::MOVING_TOWARDS_BUTTON_FROM_CENTER:
            seedingState = funcs.movingTowardsButtonFromCenter();
            break;
        case SeedingRoundState::MOVING_TOWARDS_CENTER_FROM_BUTTON:
            seedingState = funcs.movingTowardsCenterFromButton();
            break;
        case SeedingRoundState::TURNING_TOWARDS_BUTTON_CORNER_FROM_CENTER:
            seedingState = funcs.turningTowardsButtonCornerFromCenter();
            break;
        case SeedingRoundState::MOVING_TOWARDS_BUTTON_CORNER_FROM_CENTER:
            seedingState = funcs.movingTowardsButtonCornerFromCenter();
            break;
        case SeedingRoundState::TURNING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER:
            seedingState = funcs.turningTowardsStartCornerFromButtonCorner();
            break;
        case SeedingRoundState::MOVING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER:
            seedingState = funcs.movingTowardsStartCornerFromButtonCorner();
            break;
        }
        break;
    case Round::ELIMINATION:
        switch (elimState)
        {
        case EliminationRoundState::POWER_ON:
            elimState = funcs.powerOn();
            break;
        case EliminationRoundState::MOVING_BACKWARDS_FROM_STATION:
            elimState = funcs.movingBackwardsFromStation();
            break;
        case EliminationRoundState::TURNING_AWAY_FROM_WALL:
            elimState = funcs.turningAwayFromWall();
            break;
        case EliminationRoundState::MOVING_AWAY_FROM_OBSTACLES:
            elimState = funcs.movingAwayFromObstacles();
            break;
        case EliminationRoundState::MOVING_TOWARDS_NEXT_STATION:
            elimState = funcs.movingTowardsNextStation();
            break;
        case EliminationRoundState::TURNING_TOWARDS_PERPENDICULAR_BUTTON_WALL:
            elimState = funcs.turningTowardsPerpendicularButtonWall();
            break;
        case EliminationRoundState::MOVING_TOWARDS_PERPENDICULAR_BUTTON_WALL:
            elimState = funcs.movingTowardsPerpendicularButtonWall();
            break;
        case EliminationRoundState::MOVING_AWAY_FROM_PERPENDICULAR_BUTTON_WALL:
            elimState = funcs.movingAwayFromPerpendicularButtonWall();
            break;
        case EliminationRoundState::TURNING_TOWARDS_CHARGER_FROM_CORNER:
            elimState = funcs.turningTowardsChargerFromCorner();
            break;
        case EliminationRoundState::MOVING_TOWARDS_CHARGER_FROM_CORNER:
            elimState = funcs.movingTowardsChargerFromCorner();
            break;
        case EliminationRoundState::CHARGING:
            elimState = funcs.charging();
            break;
        }
        break;
    case Round::NONE:
        while (!Serial);
        Serial.println("Round not selected. Doing nothing.");
        while (true);
    }
}

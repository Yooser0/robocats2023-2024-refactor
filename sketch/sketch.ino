#include "include/utils.h"
#include "include/structs.h"
#include "include/funcs.h"

Robot robot;
Funcs funcs = Funcs(robot);
Round whatRound;
SeedingRoundState seedingState;
EliminationRoundState elimState;

void setup()
{
    whatRound = utils::getRound();

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
    switch (whatRound)
    {
    case Round::SEEDING:
        switch (seedingState)
        {
        case SeedingRoundState::TURNING_TOWARDS_START_CORNER_FROM_START:
            funcs.turningTowardsStartCornerFromStart();
            break;
        case SeedingRoundState::MOVING_TOWARDS_START_CORNER_FROM_START:
            funcs.movingTowardsStartCornerFromStart();
            break;
        case SeedingRoundState::TURNING_TOWARDS_BUTTON_CORNER_FROM_START:
            funcs.turningTowardsButtonCornerFromStart();
            break;
        case SeedingRoundState::MOVING_TOWARDS_BUTTON_CORNER_FROM_START_CORNER:
            funcs.movingTowardsButtonCornerFromStartCorner();
            break;
        case SeedingRoundState::TURNING_TOWARDS_CENTER_FROM_BUTTON_CORNER:
            funcs.turningTowardsCenterFromButtonCorner();
            break;
        case SeedingRoundState::MOVING_TOWARDS_CENTER_FROM_BUTTON_CORNER:
            funcs.movingTowardsCenterFromButtonCorner();
            break;
        case SeedingRoundState::TURNING_TOWARDS_BUTTON_FROM_CENTER:
            funcs.turningTowardsButtonFromCenter();
            break;
        case SeedingRoundState::MOVING_TOWARDS_BUTTON_FROM_CENTER:
            funcs.movingTowardsButtonFromCenter();
            break;
        case SeedingRoundState::MOVING_TOWARDS_CENTER_FROM_BUTTON:
            funcs.movingTowardsCenterFromButton();
            break;
        case SeedingRoundState::TURNING_TOWARDS_BUTTON_CORNER_FROM_CENTER:
            funcs.turningTowardsButtonCornerFromCenter();
            break;
        case SeedingRoundState::MOVING_TOWARDS_BUTTON_CORNER_FROM_CENTER:
            funcs.movingTowardsButtonCornerFromCenter();
            break;
        case SeedingRoundState::TURNING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER:
            funcs.turningTowardsStartCornerFromButtonCorner();
            break;
        case SeedingRoundState::MOVING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER:
            funcs.movingTowardsStartCornerFromButtonCorner();
            break;
        }
        break;
    case Round::ELIMINATION:
        switch (elimState)
        {
        case EliminationRoundState::POWER_ON:
            funcs.powerOn();
            break;
        case EliminationRoundState::MOVING_BACKWARDS_FROM_STATION:
            funcs.movingBackwardsFromStation();
            break;
        case EliminationRoundState::TURNING_AWAY_FROM_WALL:
            funcs.turningAwayFromWall();
            break;
        case EliminationRoundState::MOVING_AWAY_FROM_OBSTACLES:
            funcs.movingAwayFromObstacles();
            break;
        case EliminationRoundState::MOVING_TOWARDS_NEXT_STATION:
            funcs.movingTowardsNextStation();
            break;
        case EliminationRoundState::TURNING_TOWARDS_PERPENDICULAR_BUTTON_WALL:
            funcs.turningTowardsPerpendicularButtonWall();
            break;
        case EliminationRoundState::MOVING_TOWARDS_PERPENDICULAR_BUTTON_WALL:
            funcs.movingTowardsPerpendicularButtonWall();
            break;
        case EliminationRoundState::MOVING_AWAY_FROM_PERPENDICULAR_BUTTON_WALL:
            funcs.movingAwayFromPerpendicularButtonWall();
            break;
        case EliminationRoundState::TURNING_TOWARDS_CHARGER_FROM_CORNER:
            funcs.turningTowardsChargerFromCorner();
            break;
        case EliminationRoundState::MOVING_TOWARDS_CHARGER_FROM_CORNER:
            funcs.movingTowardsChargerFromCorner();
            break;
        case EliminationRoundState::CHARGING:
            funcs.charging();
            break;
        }
        break;
    case Round::NONE:
        while (!Serial);
        Serial.println("Round not selected. Doing nothing.");
        delay(LONG_MAX);
    }
}

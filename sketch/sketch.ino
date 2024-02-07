#include "include/funcs.h"
#include "include/structs.h"

Round whatRound;

SeedingRoundState seedingState;
EliminationRoundState elimState;

void setup()
{
    whatRound = funcs::getRound();

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
    switch (whatRound) {
    case Round::SEEDING:
        switch (seedingState) {
        case SeedingRoundState::TURNING_TOWARDS_START_CORNER_FROM_START:
            // TODO
            break;
        case SeedingRoundState::MOVING_TOWARDS_START_CORNER_FROM_START:
            // TODO
            break;
        case SeedingRoundState::TURNING_TOWARDS_BUTTON_CORNER_FROM_START:
            // TODO
            break;
        case SeedingRoundState::MOVING_TOWARDS_BUTTON_CORNER_FROM_START_CORNER:
            // TODO
            break;
        case SeedingRoundState::TURNING_TOWARDS_CENTER_FROM_BUTTON_CORNER:
            // TODO
            break;
        case SeedingRoundState::MOVING_TOWARDS_CENTER_FROM_BUTTON_CORNER:
            // TODO
            break;
        case SeedingRoundState::TURNING_TOWARDS_BUTTON_FROM_CENTER:
            // TODO
            break;
        case SeedingRoundState::MOVING_TOWARDS_BUTTON_FROM_CENTER:
            // TODO
            break;
        case SeedingRoundState::MOVING_TOWARDS_CENTER_FROM_BUTTON:
            // TODO
            break;
        case SeedingRoundState::TURNING_TOWARDS_BUTTON_CORNER_FROM_CENTER:
            // TODO
            break;
        case SeedingRoundState::MOVING_TOWARDS_BUTTON_CORNER_FROM_CENTER:
            // TODO
            break;
        case SeedingRoundState::TURNING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER:
            // TODO
            break;
        case SeedingRoundState::MOVING_TOWARDS_START_CORNER_FROM_BUTTON_CORNER:
            // TODO
            break;
        case SeedingRoundState::STOPPED:
            delay(LONG_MAX);
            break;
        }
        break;
    case Round::ELIMINATION:
        switch (elimState) {
        case EliminationRoundState::POWER_ON:
            // TODO
            break;
        case EliminationRoundState::MOVING_BACKWARDS_FROM_STATION:
            // TODO
            break;
        case EliminationRoundState::TURNING_AWAY_FROM_WALL:
            // TODO
            break;
        case EliminationRoundState::MOVING_AWAY_FROM_OBSTACLES:
            // TODO
            break;
        case EliminationRoundState::MOVING_TOWARDS_NEXT_STATION:
            // TODO
            break;
        case EliminationRoundState::TURNING_TOWARDS_PERPENDICULAR_BUTTON_WALL:
            // TODO
            break;
        case EliminationRoundState::MOVING_TOWARDS_PERPENDICULAR_BUTTON_WALL:
            // TODO
            break;
        case EliminationRoundState::MOVING_AWAY_FROM_PERPENDICULAR_BUTTON_WALL:
            // TODO
            break;
        case EliminationRoundState::TURNING_TOWARDS_CHARGER_FROM_CORNER:
            // TODO
            break;
        case EliminationRoundState::MOVING_TOWARDS_CHARGER_FROM_CORNER:
            // TODO
            break;
        case EliminationRoundState::CHARGING:
            // TODO
            break;
        }
        break;
    case Round::NONE:
        while (!Serial);
        Serial.println("Round not selected. Doing nothing.");
        delay(LONG_MAX);
    }
}

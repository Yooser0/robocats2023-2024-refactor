#include "include/funcs.h"
#include "include/structs.h"

Funcs::Funcs(Robot& robot) : robot(robot) {}

// Seeding round implementation

SeedingRoundState Funcs::turningTowardsStartCornerFromStart() {
    // TODO
    return SeedingRoundState::STOPPED;
}

SeedingRoundState Funcs::movingTowardsStartCornerFromStart() {
    // TODO
    return SeedingRoundState::STOPPED;
}

SeedingRoundState Funcs::turningTowardsButtonCornerFromStart() {
    // TODO
    return SeedingRoundState::STOPPED;
}

SeedingRoundState Funcs::movingTowardsbuttonCornerFromStartCorner() {
    // TODO
    return SeedingRoundState::STOPPED;
}

SeedingRoundState Funcs::turningTowardsCenterFromButtonCorner() {
    // TODO
    return SeedingRoundState::STOPPED;
}

SeedingRoundState Funcs::movingTowardsCenterFromButtonCorner() {
    // TODO
    return SeedingRoundState::STOPPED;
}

SeedingRoundState Funcs::turningTowardsButtonFromCenter() {
    // TODO
    return SeedingRoundState::STOPPED;
}

SeedingRoundState Funcs::movingTowardsButtonFromCenter() {
    // TODO
    return SeedingRoundState::STOPPED;
}

SeedingRoundState Funcs::movingTowardsCenterFromButton() {
    // TODO
    return SeedingRoundState::STOPPED;
}

SeedingRoundState Funcs::turningTowardsButtonCornerFromCenter() {
    // TODO
    return SeedingRoundState::STOPPED;
}

SeedingRoundState Funcs::movingTowardsButtonCornerFromCenter() {
    // TODO
    return SeedingRoundState::STOPPED;
}

SeedingRoundState Funcs::turningTowardsStartCornerFromButtonCorner() {
    // TODO
    return SeedingRoundState::STOPPED;
}

SeedingRoundState Funcs::movingTowardsStartCornerFromButtonCorner() {
    // TODO
    return SeedingRoundState::STOPPED;
}

// Elimination round implementation

EliminationRoundState Funcs::powerOn() {
    // TODO
    return EliminationRoundState::STOPPED;
}

EliminationRoundState Funcs::movingBackwardsFromStation() {
    // TODO
    return EliminationRoundState::STOPPED;
}

EliminationRoundState Funcs::turningAwayFromWall() {
    // TODO
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

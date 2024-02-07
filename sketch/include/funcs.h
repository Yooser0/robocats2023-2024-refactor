#ifndef FUNCS
#define FUNCS

#include "structs.h"

class Funcs
{
    Robot& robot;
public:
    Funcs(Robot& robot);

    SeedingRoundState turningTowardsStartCornerFromStart();
    SeedingRoundState movingTowardsStartCornerFromStart();
    SeedingRoundState turningTowardsButtonCornerFromStart();
    SeedingRoundState movingTowardsButtonCornerFromStartCorner();
    SeedingRoundState turningTowardsCenterFromButtonCorner();
    SeedingRoundState movingTowardsCenterFromButtonCorner();
    SeedingRoundState turningTowardsButtonFromCenter();
    SeedingRoundState movingTowardsButtonFromCenter();
    SeedingRoundState movingTowardsCenterFromButton();
    SeedingRoundState turningTowardsButtonCornerFromCenter();
    SeedingRoundState movingTowardsButtonCornerFromCenter();
    SeedingRoundState turningTowardsStartCornerFromButtonCorner();
    SeedingRoundState movingTowardsStartCornerFromButtonCorner();

    EliminationRoundState powerOn();
    EliminationRoundState movingBackwardsFromStation();
    EliminationRoundState turningAwayFromWall();
    EliminationRoundState movingAwayFromObstacles();
    EliminationRoundState movingTowardsNextStation();
    EliminationRoundState turningTowardsPerpendicularButtonWall();
    EliminationRoundState movingTowardsPerpendicularButtonWall();
    EliminationRoundState movingAwayFromPerpendicularButtonWall();
    EliminationRoundState turningTowardsChargerFromCorner();
    EliminationRoundState movingTowardsChargerFromCorner();
    EliminationRoundState charging();
};

#endif

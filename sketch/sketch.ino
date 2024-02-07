#include "include/funcs.h"
#include "include/structs.h"

Round whatRound;

void setup()
{
    whatRound = funcs::getRound();
}

void loop()
{
    switch (whatRound) {
    case Round::SEEDING:
        // TODO
        break;
    case Round::ELIMINATION:
        // TODO
        break;
    case Round::NONE:
        while (!Serial);
        Serial.println("Round not selected. Doing nothing.");
        delay(LONG_MAX);
    }
}

#ifndef LEDS_H
#define LEDS_H

#include <Adafruit_MCP23X17.h>
#include "LevelSelection.h"
#include "BranchSelection.h"

#define NUM_ROWS 4
#define NUM_COLS 12

/**
 * Columns are wired to {fill in the blank}
 * Rows are wired to {fill in the blank}
 */
#define ROW_ON HIGH
#define ROW_OFF LOW
#define COL_ON LOW
#define COL_OFF HIGH

class Leds
{
public:
    // ! It is CRUCIAL that these are in order of LevelSelection::Level starting at L1 and ending at L4
    int ROW_PINS[NUM_ROWS] = {0, 1, 2, 3};
    // ! It is CRUCIAL that these are in order of BranchSelection::Branch starting at A and ending at L
    int COL_PINS[NUM_COLS] = {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

    Leds::Leds(byte mcpI2CAddress);

    void init();
    void update(LevelSelection::Level lvl, BranchSelection::Branch branch);

private:
    void resetLeds();
    int getRowPin(LevelSelection::Level lvl);
    int getColPin(BranchSelection::Branch branch);

    byte m_mcpI2CAddress;
    Adafruit_MCP23X17 m_mcp;
};
#endif // LEDS_H

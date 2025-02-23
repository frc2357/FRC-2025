#include "Leds.h"

Leds::Leds(byte mcpI2CAddress) : m_mcpI2CAddress(mcpI2CAddress)
{
}

void Leds::init()
{
    if (!m_mcp.begin_I2C(m_mcpI2CAddress))
    {
        Serial.println("Failed to establish communication with the Led MCP23017 I2C device");
        while (1)
            ;
    }

    resetLeds();
}

void Leds::update(LevelSelection::Level lvl, BranchSelection::Branch branch)
{
    int rowPin = getRowPin(lvl);
    int colPin = getColPin(branch);

    // TODO: Do whatever cool effects we want to do with this
    if (rowPin != -1 && colPin != -1)
    {
        digitalWrite(rowPin, ROW_ON);
        digitalWrite(colPin, COL_ON);
    }
}

void Leds::resetLeds()
{
    for (int rowPin : ROW_PINS)
    {
        pinMode(rowPin, OUTPUT);
        digitalWrite(rowPin, ROW_OFF);
    }

    for (int colPin : COL_PINS)
    {
        pinMode(colPin, OUTPUT);
        digitalWrite(colPin, COL_OFF);
    }
}

int Leds::getRowPin(LevelSelection::Level lvl)
{
    if (lvl == -1)
    {
        return -1;
    }

    return ROW_PINS[lvl];
}

int Leds::getColPin(BranchSelection::Branch branch)
{
    if (branch == -1)
    {
        return -1;
    }

    int len = sizeof(BranchSelection::PINS) / sizeof(BranchSelection::PINS[0]);
    for (int i = 0; i < len; i++)
    {
        if (BranchSelection::PINS[i] == branch)
        {
            return COL_PINS[i];
        }
    }
}
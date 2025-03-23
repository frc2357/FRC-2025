#include "Leds.h"

Leds::Leds(byte mcpI2CAddress) : m_mcpI2CAddress(mcpI2CAddress)
{
}

void Leds::init()
{
    if (!m_mcp.begin_I2C(m_mcpI2CAddress))
    {
        Serial.print("Failed to establish communication with the Led MCP23017 I2C device (0x");
        Serial.print(m_mcpI2CAddress, 16);
        Serial.println(")");
        while (1)
            ;
    }

    for (int rowPin : ROW_PINS)
    {
        m_mcp.pinMode(rowPin, OUTPUT);
        m_mcp.digitalWrite(rowPin, ROW_OFF);
    }

    for (int colPin : COL_PINS)
    {
        m_mcp.pinMode(colPin, OUTPUT);
        m_mcp.digitalWrite(colPin, COL_OFF);
    }
}

void Leds::update(LevelSelection::Level lvl, BranchSelection::Branch branch)
{
    int rowPin = getRowPin(lvl);
    int colPin = getColPin(branch);

    if (rowPin != m_currentRowPin || colPin != m_currentColPin)
    {
        resetLeds();
    }

    if (rowPin != -1 && colPin != -1)
    {
        m_mcp.digitalWrite(rowPin, ROW_ON);
        m_mcp.digitalWrite(colPin, COL_ON);
    }
    else if (rowPin != -1)
    {
        m_mcp.digitalWrite(rowPin, ROW_ON);

        int animationFrameMillis = millis() % ANIMATION_FRAME_DURATION_MILLIS;

        for (int col : COL_PINS)
        {
            m_mcp.digitalWrite(col, COL_ON);
        }
    }
    else if (colPin != -1)
    {
        m_mcp.digitalWrite(colPin, COL_ON);
        for (int row : ROW_PINS)
        {
            m_mcp.digitalWrite(row, ROW_ON);
        }
    }
    else
    {
        resetLeds();
    }

    m_currentRowPin = rowPin;
    m_currentColPin = colPin;
}

void Leds::resetLeds()
{
    for (int rowPin : ROW_PINS)
    {
        m_mcp.digitalWrite(rowPin, ROW_OFF);
    }

    for (int colPin : COL_PINS)
    {
        m_mcp.digitalWrite(colPin, COL_OFF);
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
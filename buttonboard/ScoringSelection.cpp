#include "ScoringSelection.h"

ScoringSelection::ScoringSelection(byte leftKeypadI2CAddress, byte rightKeypadI2CAddress) : m_leftKeypadAddress(leftKeypadI2CAddress), m_rightKeypadAddress(rightKeypadI2CAddress)
{
}

void ScoringSelection::init()
{
    if (!m_leftKeypad.begin(m_leftKeypadAddress))
    {
        Serial.println("Failed to establish communication with the Left Scoring Level Selection Keypad I2C device");
        while (1)
            ;
    }

    if (!m_rightKeypad.begin(m_rightKeypadAddress))
    {
        Serial.println("Failed to establish communication with the Right Scoring Level Selection Keypad I2C device");
        while (1)
            ;
    }

    // Clear leds
    for (uint8_t i = 0; i < m_leftKeypad.pixels.numPixels(); i++)
    {
        m_leftKeypad.pixels.setPixelColor(i, COLOR_OFF);
        m_rightKeypad.pixels.setPixelColor(i, COLOR_OFF);
    }
    m_leftKeypad.pixels.show();
    m_rightKeypad.pixels.show();
}

void ScoringSelection::update()
{
    uint8_t leftState = m_leftKeypad.read();
    uint8_t rightState = m_rightKeypad.read();

    if (leftState != 0 || rightState != 0)
    {
        // Find log_2 of the combined state
        // This will basically tell us the position of the most significant bit that is set to 1
        setCurrentSelection(log2(leftState | rightState));
    }
}

void ScoringSelection::setCurrentSelection(int selection)
{
    if (selection == m_currentSelection)
    {
        m_currentSelection = -1;
        m_leftKeypad.pixels.setPixelColor(selection, COLOR_OFF);
        m_rightKeypad.pixels.setPixelColor(selection, COLOR_OFF);
    }
    else
    {
        m_currentSelection = selection;
        m_leftKeypad.pixels.setPixelColor(selection, COLOR_ON);
        m_rightKeypad.pixels.setPixelColor(selection, COLOR_ON);
    }

    m_leftKeypad.pixels.show();
    m_rightKeypad.pixels.show();
}
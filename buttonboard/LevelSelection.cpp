#include "LevelSelection.h"

LevelSelection::LevelSelection(byte leftKeypadI2CAddress, byte rightKeypadI2CAddress)
    : m_leftKeypadI2CAddress(leftKeypadI2CAddress),
      m_rightKeypadI2CAddress(rightKeypadI2CAddress)
{
}

void LevelSelection::init()
{
    if (!m_leftKeypad.begin(m_leftKeypadI2CAddress))
    {
        Serial.println("Failed to establish communication with the left Scoring Level Selection Keypad I2C device");
        while (true)
            ;
    }

    if (!m_rightKeypad.begin(m_rightKeypadI2CAddress))
    {
        Serial.println("Failed to establish communication with the right Scoring Level Selection Keypad I2C device");
        while (true)
            ;
    }

    for (uint8_t i = 0; i < NUM_BUTTONS; i++)
    {
        setLedState(i, false);
    }
    showLEDs();
}

void LevelSelection::update()
{
    if (millis() < m_lastEventMillis + DEBOUNCE_MILLIS)
    {
        return;
    };

    uint8_t state = m_leftKeypad.read() | m_rightKeypad.read();

    if (state != 0 && m_prevState != state)
    {
        // Calculate log base 2 of the combined state
        // This gives us the most significant bit that is set to 1
        int sel = log_2(state);

        setLedState(m_selection, false);
        XInput.release(SCORING_LEVEL_XBOX_BUTTONS[m_selection]);

        if (sel == m_selection)
        {
            m_selection = Level::NONE;
        }
        else
        {
            m_selection = sel;
            setLedState(m_selection, true);
            XInput.press(SCORING_LEVEL_XBOX_BUTTONS[m_selection]);
        }

        showLEDs();

        m_lastEventMillis = millis();
    }
    m_prevState = state;
}

LevelSelection::Level LevelSelection::getSelection()
{
    return m_selection;
}

void LevelSelection::setLedState(int index, bool on)
{
    m_leftKeypad.pixels.setPixelColor(index, on ? COLOR_ON : COLOR_OFF);
    m_rightKeypad.pixels.setPixelColor(index, on ? COLOR_ON : COLOR_OFF);
}

void LevelSelection::showLEDs()
{
    m_leftKeypad.pixels.show();
    m_rightKeypad.pixels.show();
}
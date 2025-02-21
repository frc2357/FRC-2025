#ifndef LEVEL_SELECTION_H
#define LEVEL_SELECTION_H

#include <Adafruit_NeoKey_1x4.h>
#include <XInput.h>

#define COLOR_ON 0xFFFFFFFF
#define COLOR_OFF 0x00000000

#define NUM_BUTTONS 4

#define log_2(x) (log(x) / log(2))

const XInputControl SCORING_LEVEL_XBOX_BUTTONS[NUM_BUTTONS] = {
    DPAD_UP,
    DPAD_RIGHT,
    DPAD_DOWN,
    DPAD_LEFT,
};

class LevelSelection
{
public:
    // Int values are the index of the button returned from keypad.read()
    enum Level
    {
        NONE = -1,
        L1 = 0,
        L2 = 1,
        L3 = 2,
        L4 = 3
    };

    LevelSelection(byte leftKeypadI2CAddress, byte rightKeypadI2CAddress);
    void init();
    void update();

private:
    void setLedState(int index, bool on);
    void showLEDs();

    byte m_leftKeypadI2CAddress, m_rightKeypadI2CAddress;
    Adafruit_NeoKey_1x4 m_leftKeypad, m_rightKeypad;
    LevelSelection::Level m_selection = LevelSelection::Level::NONE;
    bool m_hasReleased = true;
};

#endif // LEVEL_SELECTION_H
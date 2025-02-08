#ifndef SCORING_SELECTION_H
#define SCORING_SELECTION_H

#include "Adafruit_NeoKey_1x4.h"
#include <XInput.h>

#define COLOR_ON 0xFFFFFFFF
#define COLOR_OFF 0x00000000

// Used to determine which binary bit of keypad state is selected
#define log2(x) (log(x) / log(2))

const XInputControl SCORING_LEVEL_CONTROLLER_BUTTONS[4] = {
    DPAD_UP,
    DPAD_RIGHT,
    DPAD_DOWN,
    DPAD_LEFT,
};

class ScoringSelection
{
public:
    static void init(byte leftKeypadI2CAddress, byte rightKeypadI2CAddress, byte keypadInterruptPin);

private:
    static void updateScoringLevelSelection();
    static void setKeypadLedState(int index, bool on);

    static Adafruit_NeoKey_1x4 leftKeypad, rightKeypad;
    static int scoringLevelSelection;
};

#endif // SCORING_SELECTION_H
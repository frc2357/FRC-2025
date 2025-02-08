#include "LevelSelection.h"

Adafruit_NeoKey_1x4 LevelSelection::leftKeypad;
Adafruit_NeoKey_1x4 LevelSelection::rightKeypad;

LevelSelection::Level LevelSelection::selection = LevelSelection::Level::NONE;

void LevelSelection::init(byte leftKeypadI2CAddress, byte rightKeypadI2CAddress, byte interruptPin)
{
    if (!LevelSelection::leftKeypad.begin(leftKeypadI2CAddress))
    {
        Serial.println("Failed to establish communication with the left Scoring Level Selection Keypad I2C device");
        while (true)
            ;
    }

    if (!LevelSelection::rightKeypad.begin(rightKeypadI2CAddress))
    {
        Serial.println("Failed to establish communication with the right Scoring Level Selection Keypad I2C device");
        while (true)
            ;
    }

    for (uint8_t i = 0; i < NUM_BUTTONS; i++)
    {
        LevelSelection::setLedState(i, false);
    }
    LevelSelection::showLEDs();

    // Keypad interrupts enabled by default. Pin goes LOW when a button pressed
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), LevelSelection::updateSelection, FALLING);
}

void LevelSelection::updateSelection()
{
    uint8_t state = LevelSelection::leftKeypad.read() | LevelSelection::rightKeypad.read();

    if (state != 0)
    {
        // Calculate log base 2 of the combined state
        // This gives us the most significant bit that is set to 1
        int sel = log_2(state);

        bool isSelecting = sel == LevelSelection::selection;

        LevelSelection::selection = static_cast<LevelSelection::Level>(isSelecting ? selection : -1);
        LevelSelection::setLedState(selection, isSelecting);
        XInput.setButton(SCORING_LEVEL_XBOX_BUTTONS[selection], isSelecting);

        LevelSelection::showLEDs();
    }
}

void LevelSelection::setLedState(int index, bool on)
{
    LevelSelection::leftKeypad.pixels.setPixelColor(index, on ? COLOR_ON : COLOR_OFF);
    LevelSelection::rightKeypad.pixels.setPixelColor(index, on ? COLOR_ON : COLOR_OFF);
}

void LevelSelection::showLEDs()
{
    LevelSelection::leftKeypad.pixels.show();
    LevelSelection::rightKeypad.pixels.show();
}
#include "ScoringSelection.h"

Adafruit_NeoKey_1x4 ScoringSelection::leftKeypad;
Adafruit_NeoKey_1x4 ScoringSelection::rightKeypad;

int ScoringSelection::scoringLevelSelection = -1;

void ScoringSelection::init(byte leftKeypadI2CAddress, byte rightKeypadI2CAddress, byte keypadInterruptPin)
{
    if (!ScoringSelection::leftKeypad.begin(leftKeypadI2CAddress))
    {
        Serial.println("Failed to establish communication with the Left Scoring Level Selection Keypad I2C device");
        while (1)
            ;
    }

    if (!ScoringSelection::rightKeypad.begin(rightKeypadI2CAddress))
    {
        Serial.println("Failed to establish communication with the Right Scoring Level Selection Keypad I2C device");
        while (1)
            ;
    }

    // Clear leds
    for (uint8_t i = 0; i < ScoringSelection::leftKeypad.pixels.numPixels(); i++)
    {
        ScoringSelection::setKeypadLedState(i, false);
    }

    // Interrupts are enabled by default on the keypads. Pin goes LOW when any button pressed
    pinMode(keypadInterruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(keypadInterruptPin), ScoringSelection::updateScoringLevelSelection, FALLING);
}

void ScoringSelection::updateScoringLevelSelection()
{
    uint8_t leftState = ScoringSelection::leftKeypad.read();
    uint8_t rightState = ScoringSelection::rightKeypad.read();

    if (leftState != 0 || rightState != 0)
    {
        // Find log_2 of the combined state
        // This will basically tell us the position of the most significant bit that is set to 1
        int selection = log2(leftState | rightState);
        if (selection == ScoringSelection::scoringLevelSelection)
        {
            XInput.release(SCORING_LEVEL_CONTROLLER_BUTTONS[ScoringSelection::scoringLevelSelection]);
            ScoringSelection::scoringLevelSelection = -1;
            ScoringSelection::setLedState(selection, false);
        }
        else
        {
            ScoringSelection::scoringLevelSelection = selection;
            ScoringSelection::setKeypadLedState(selection, true);
            XInput.press(SCORING_LEVEL_CONTROLLER_BUTTONS[ScoringSelection::scoringLevelSelection]);
        }
    }
}

void ScoringSelection::setKeypadLedState(int index, bool on)
{
    ScoringSelection::leftKeypad.pixels.setPixelColor(index, on ? COLOR_ON : COLOR_OFF);
    ScoringSelection::rightKeypad.pixels.setPixelColor(index, on ? COLOR_ON : COLOR_OFF);
    ScoringSelection::leftKeypad.pixels.show();
    ScoringSelection::rightKeypad.pixels.show();
}
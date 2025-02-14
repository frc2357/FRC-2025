#ifndef PANIC_CONTROLS_H
#define PANIC_CONTROLS_H

#include <Adafruit_MCP23X17.h>
#include <Xinput.h>

#define NUM_BUTTONS 12

#define POT_MIN_VALUE 0
#define POT_MAX_VALUE 1023

#define ROLLER_POT_PIN A0
#define MOVEMENT_POT_PIN A1

class PanicControls
{
public:
    // integer values correspond to pin numbers
    enum MechanismControl
    {
        NONE = -1,
        CORAL_FORWARD = 0,
        CORAL_REVERSE = 1,
        ALGAE_FORWARD = 2,
        ALGAE_REVERSE = 3,
        ELEVATOR_UP = 4,
        ELEVATOR_DOWN = 5,
        LATERATOR_FORWARD = 6,
        LATERATOR_REVERSE = 7,
        ALGAE_OUT = 8,
        ALGAE_IN = 9,
        CLIMBER_OUT = 10,
        CLIMBER_IN = 11,
    };
    static int pins[NUM_BUTTONS];

    static void init(byte mcpI2CAddress, byte intPin);

    static void onPinActivated(int pin);
    static void onPinDeactivated(int pin);

private:
    static void setXboxControlsForMechanism(PanicControls::MechanismControl mechanism, int potVal);

    static Adafruit_MCP23X17 mcp;
    static PanicControls::MechanismControl selection;
};

#endif // PANIC_CONTROLS_H
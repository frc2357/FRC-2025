#ifndef PANIC_CONTROLS_H
#define PANIC_CONTROLS_H

#include <Adafruit_MCP23X17.h>
#include <Xinput.h>

#define NUM_BUTTONS 12
#define BUTTON_PRESSED_STATE LOW
#define INTERRUPT_SET_STATE LOW

// Sliders are installed upside down
#define POTS_REVERSED true
#define POT_MIN_VALUE 0
#define POT_MAX_VALUE 1023

#define ROLLER_POT_PIN A4
#define MOVEMENT_POT_PIN A3

// Button to indicate if the roller mechanism panic controls are reversed or not
#define ROLLER_NEGATIVE_INDICATOR_BUTTON XInputControl::BUTTON_R3

class PanicControls
{
public:
    // integer values correspond to pin numbers
    enum MechanismControl
    {
        NONE = -1,
        CORAL_FORWARD = 8,
        CORAL_REVERSE = 9,
        ALGAE_FORWARD = 10,
        ALGAE_REVERSE = 11,
        ELEVATOR_UP = 7,
        ELEVATOR_DOWN = 6,
        LATERATOR_FORWARD = 5,
        LATERATOR_REVERSE = 4,
        ALGAE_OUT = 3,
        ALGAE_IN = 2,
        CLIMBER_OUT = 1,
        CLIMBER_IN = 0,
    };
    int PINS[NUM_BUTTONS] = {
        MechanismControl::CORAL_FORWARD,
        MechanismControl::CORAL_REVERSE,
        MechanismControl::ALGAE_FORWARD,
        MechanismControl::ALGAE_REVERSE,
        MechanismControl::ELEVATOR_UP,
        MechanismControl::ELEVATOR_DOWN,
        MechanismControl::LATERATOR_FORWARD,
        MechanismControl::LATERATOR_REVERSE,
        MechanismControl::ALGAE_OUT,
        MechanismControl::ALGAE_IN,
        MechanismControl::CLIMBER_OUT,
        MechanismControl::CLIMBER_IN,
    };

    PanicControls::PanicControls(byte mcpI2CAddress, byte intPin);

    void init();
    void update();

private:
    void setXboxControlsForMechanism(PanicControls::MechanismControl mechanism);
    void resetJoysticks();
    int readPot(byte potPin);

    byte m_mcpI2CAddress, m_interruptPin;
    Adafruit_MCP23X17 m_mcp;
    PanicControls::MechanismControl m_selection = -1;
    bool m_joysticksReset;
};

#endif // PANIC_CONTROLS_H
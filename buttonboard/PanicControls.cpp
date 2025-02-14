#include "PanicControls.h"

int PanicControls::pins[NUM_BUTTONS] = {
    PanicControls::MechanismControl::CORAL_FORWARD,
    PanicControls::MechanismControl::CORAL_REVERSE,
    PanicControls::MechanismControl::ALGAE_FORWARD,
    PanicControls::MechanismControl::ALGAE_REVERSE,
    PanicControls::MechanismControl::ELEVATOR_UP,
    PanicControls::MechanismControl::ELEVATOR_DOWN,
    PanicControls::MechanismControl::LATERATOR_FORWARD,
    PanicControls::MechanismControl::LATERATOR_REVERSE,
    PanicControls::MechanismControl::ALGAE_OUT,
    PanicControls::MechanismControl::ALGAE_IN,
    PanicControls::MechanismControl::CLIMBER_OUT,
    PanicControls::MechanismControl::CLIMBER_IN,
};

PanicControls::MechanismControl PanicControls::selection;

void PanicControls::init(byte mcpI2CAddress, byte intPin)
{
    if (!PanicControls::mcp.begin_I2C(mcpI2CAddress))
    {
        Serial.println("Failed to establish communication with the Panic Controls MCP23017 I2C device");
        while (1)
            ;
    }

    pinMode(intPin, INPUT_PULLUP);

    PanicControls::mcp.setupInterrupts(true, false, LOW);

    for (int pin : PanicControls::pins)
    {
        PanicControls::mcp.pinMode(pin, INPUT_PULLUP);
        PanicControls::mcp.setupInterruptPin(pin, CHANGE);
    }

    XInput.setTriggerRange(-POT_MAX_VALUE, POT_MAX_VALUE);
    XInput.setJoystickRange(-POT_MAX_VALUE, POT_MAX_VALUE);
}

void PanicControls::onPinActivated(int pin)
{
    PanicControls::selection = static_cast<PanicControls::MechanismControl>(pin);
}

void PanicControls::onPinDeactivated(int pin)
{
    setXboxControlsForMechanism(PanicControls::selection, 0);
    PanicControls::selection = PanicControls::MechanismControl::NONE;
}

void PanicControls::setXboxControlsForMechanism(PanicControls::MechanismControl mechanism, int potVal)
{
    switch (mechanism)
    {
    case CORAL_FORWARD:
        XInput.setTrigger(XInputControl::TRIGGER_RIGHT, potVal);
        break;
    case CORAL_REVERSE:
        XInput.setTrigger(XInputControl::TRIGGER_RIGHT, -potVal);
        break;
    case ALGAE_FORWARD:
        XInput.setTrigger(XInputControl::TRIGGER_LEFT, potVal);
        break;
    case ALGAE_REVERSE:
        XInput.setTrigger(XInputControl::TRIGGER_LEFT, -potVal);
        break;
    case ELEVATOR_UP:
        XInput.setJoystickY(XInputControl::JOY_RIGHT, potVal);
        break;
    case ELEVATOR_DOWN:
        XInput.setJoystickY(XInputControl::JOY_RIGHT, -potVal);
        break;
    case LATERATOR_FORWARD:
        XInput.setJoystickX(XInputControl::JOY_RIGHT, potVal);
        break;
    case LATERATOR_REVERSE:
        XInput.setJoystickX(XInputControl::JOY_RIGHT, -potVal);
        break;
    case ALGAE_OUT:
        XInput.setJoystickX(XInputControl::JOY_LEFT, potVal);
        break;
    case ALGAE_IN:
        XInput.setJoystickX(XInputControl::JOY_LEFT, -potVal);
        break;
    case CLIMBER_OUT:
        XInput.setJoystickY(XInputControl::JOY_LEFT, potVal);
        break;
    case CLIMBER_IN:
        XInput.setJoystickY(XInputControl::JOY_LEFT, -potVal);
        break;
    }
}
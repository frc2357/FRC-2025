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

Adafruit_MCP23X17 PanicControls::mcp;

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

void PanicControls::update()
{
    if (PanicControls::selection == PanicControls::MechanismControl::NONE)
    {
        PanicControls::clearXboxAxes();
    }
    else
    {
        PanicControls::setXboxControlsForMechanism(PanicControls::selection);
    }
}

void PanicControls::onPinActivated(int pin)
{
    PanicControls::selection = static_cast<PanicControls::MechanismControl>(pin);
}

void PanicControls::onPinDeactivated(int pin)
{
    PanicControls::selection = PanicControls::MechanismControl::NONE;
}

void PanicControls::setXboxControlsForMechanism(PanicControls::MechanismControl mechanism)
{
    switch (mechanism)
    {
    case CORAL_FORWARD:
        XInput.setTrigger(XInputControl::TRIGGER_RIGHT, analogRead(ROLLER_POT_PIN));
        break;
    case CORAL_REVERSE:
        XInput.setTrigger(XInputControl::TRIGGER_RIGHT, -analogRead(ROLLER_POT_PIN));
        break;
    case ALGAE_FORWARD:
        XInput.setTrigger(XInputControl::TRIGGER_LEFT, analogRead(ROLLER_POT_PIN));
        break;
    case ALGAE_REVERSE:
        XInput.setTrigger(XInputControl::TRIGGER_LEFT, -analogRead(ROLLER_POT_PIN));
        break;
    case ELEVATOR_UP:
        XInput.setJoystickY(XInputControl::JOY_RIGHT, analogRead(MOVEMENT_POT_PIN));
        break;
    case ELEVATOR_DOWN:
        XInput.setJoystickY(XInputControl::JOY_RIGHT, -analogRead(MOVEMENT_POT_PIN));
        break;
    case LATERATOR_FORWARD:
        XInput.setJoystickX(XInputControl::JOY_RIGHT, analogRead(MOVEMENT_POT_PIN));
        break;
    case LATERATOR_REVERSE:
        XInput.setJoystickX(XInputControl::JOY_RIGHT, -analogRead(MOVEMENT_POT_PIN));
        break;
    case ALGAE_OUT:
        XInput.setJoystickX(XInputControl::JOY_LEFT, analogRead(MOVEMENT_POT_PIN));
        break;
    case ALGAE_IN:
        XInput.setJoystickX(XInputControl::JOY_LEFT, -analogRead(MOVEMENT_POT_PIN));
        break;
    case CLIMBER_OUT:
        XInput.setJoystickY(XInputControl::JOY_LEFT, analogRead(MOVEMENT_POT_PIN));
        break;
    case CLIMBER_IN:
        XInput.setJoystickY(XInputControl::JOY_LEFT, -analogRead(MOVEMENT_POT_PIN));
        break;
    }
}

void PanicControls::clearXboxAxes()
{
    XInput.setTrigger(XInputControl::TRIGGER_RIGHT, 0);
    XInput.setTrigger(XInputControl::TRIGGER_LEFT, 0);
    XInput.setJoystickY(XInputControl::JOY_RIGHT, 0);
    XInput.setJoystickX(XInputControl::JOY_RIGHT, 0);
    XInput.setJoystickX(XInputControl::JOY_LEFT, 0);
    XInput.setJoystickY(XInputControl::JOY_LEFT, 0);
}
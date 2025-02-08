#include "PanicControls.h"

Adafruit_MCP23X17 PanicControls::mcp;

PanicControls::Mechanism PanicControls::selectedMechanism = PanicControls::Mechanism::NONE;
bool PanicControls::mechanismReversed = false;

uint64_t PanicControls::lastInterruptMillis = 0;

void PanicControls::init(byte mcpI2CAddress, byte intPin)
{
    if (!PanicControls::mcp.begin_I2C(mcpI2CAddress))
    {
        Serial.println("Failed to establish communication with the Panic Controls MCP23017 I2C device");
        while (1)
            ;
    }

    pinMode(intPin, INPUT_PULLUP);
    // mirror INTA/B so we only need to connect to one int pin
    PanicControls::mcp.setupInterrupts(true, false, LOW);

    for (int pin = 0; pin < PIN_COUNT; pin++)
    {
        PanicControls::mcp.pinMode(pin, INPUT);
        // Enable interrupts when `pin` changes values
        PanicControls::mcp.setupInterruptPin(pin, CHANGE);
    }

    XInput.setTriggerRange(POT_MIN_VALUE, POT_MAX_VALUE);
    XInput.setJoystickRange(POT_MIN_VALUE, POT_MAX_VALUE);

    attachInterrupt(digitalPinToInterrupt(intPin), PanicControls::update, FALLING);
}

void PanicControls::update()
{
    if (millis() > DEBOUNCE_TIME_MILLIS + PanicControls::lastInterruptMillis)
    {
        // Each bit of `state` is the value of the pin at that index
        // e.g. 0b0001 means that pins 1-3 are low and pin 0 is high
        uint8_t pin = PanicControls::mcp.getLastInterruptPin();
        uint16_t state = PanicControls::mcp.getCapturedInterrupt();
        // Right shift to get `pin`s bit to index 0
        // Bitwise anding this with 1 tells us if `pin` is high or low
        bool pinValue = 1 && state >> pin;

        if (pinValue)
        {
            uint8_t mechIndex = pin / 2; // 2 for the 2 directions
            PanicControls::selectedMechanism = static_cast<PanicControls::Mechanism>(mechIndex);
            PanicControls::mechanismReversed = pin % 2;
        }
        else
        {
            PanicControls::selectedMechanism = PanicControls::Mechanism::NONE;
            PanicControls::mechanismReversed = false;
        }

        PanicControls::lastInterruptMillis = millis();

        if (PanicControls::selectedMechanism != PanicControls::Mechanism::NONE)
        {
            PanicControls::setControllerAxes();
        }
    }
}

void PanicControls::setControllerAxes()
{
    switch (PanicControls::selectedMechanism)
    {
    case CORAL_INTAKE_ROLLER:
        PanicControls::setCoralRollers(true);
        break;
    case ALGAE_INTAKE_ROLLER:
        PanicControls::setAlgaeRollers(true);
        break;
    case ELEVATOR:
        PanicControls::setElevator(true);
        break;
    case LATERATOR:
        PanicControls::setLaterator(true);
        break;
    case ALGAE_PIVOT:
        PanicControls::setAlgaePivot(true);
        break;
    case CLIMBER:
        PanicControls::setClimber(true);
        break;
    default:
        // Clear all axes
        PanicControls::setCoralRollers(false);
        PanicControls::setAlgaeRollers(false);
        PanicControls::setElevator(false);
        PanicControls::setLaterator(false);
        PanicControls::setAlgaePivot(false);
        PanicControls::setClimber(false);
        break;
    }
}

void PanicControls::setCoralRollers(bool on)
{
    XInput.setTrigger(XInputControl::TRIGGER_RIGHT, on * analogRead(ROLLER_POT_PIN));
    XInput.setButton(XInputControl::BUTTON_R3, on && PanicControls::mechanismReversed);
}

void PanicControls::setAlgaeRollers(bool on)
{
    XInput.setTrigger(XInputControl::TRIGGER_LEFT, on * analogRead(ROLLER_POT_PIN));
    XInput.setButton(XInputControl::BUTTON_L3, on && PanicControls::mechanismReversed);
}

void PanicControls::setElevator(bool on)
{
    uint16_t val = on ? analogRead(MOVEMENT_POT_PIN) : 0;
    XInput.setJoystickY(XInputControl::JOY_RIGHT, val, on && PanicControls::mechanismReversed);
}

void PanicControls::setLaterator(bool on)
{
    uint16_t val = on ? analogRead(MOVEMENT_POT_PIN) : 0;
    XInput.setJoystickX(XInputControl::JOY_RIGHT, val, on && PanicControls::mechanismReversed);
}

void PanicControls::setAlgaePivot(bool on)
{
    uint16_t val = on ? analogRead(MOVEMENT_POT_PIN) : 0;
    XInput.setJoystickX(XInputControl::JOY_LEFT, val, on && PanicControls::mechanismReversed);
}

void PanicControls::setClimber(bool on)
{
    uint16_t val = on ? analogRead(MOVEMENT_POT_PIN) : 0;
    XInput.setJoystickY(XInputControl::JOY_LEFT, val, on && PanicControls::mechanismReversed);
}
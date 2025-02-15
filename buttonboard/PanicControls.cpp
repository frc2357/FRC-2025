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

int PanicControls::interruptPin;
Adafruit_MCP23X17 PanicControls::mcp;
PanicControls::MechanismControl PanicControls::selection = -1;

void PanicControls::init(byte mcpI2CAddress, byte intPin)
{
  if (!PanicControls::mcp.begin_I2C(mcpI2CAddress))
  {
    Serial.println("Failed to establish communication with the Panic Controls MCP23017 I2C device");
    while (1)
      ;
  }

  PanicControls::interruptPin = intPin;
  pinMode(intPin, INPUT_PULLUP);
  PanicControls::mcp.setupInterrupts(true, false, INTERRUPT_SET_STATE);

  for (int pin : PanicControls::pins)
  {
    PanicControls::mcp.pinMode(pin, INPUT_PULLUP);
    PanicControls::mcp.setupInterruptPin(pin, BUTTON_PRESSED_STATE);
  }

  XInput.setTriggerRange(POT_MIN_VALUE, POT_MAX_VALUE);
  XInput.setJoystickRange(-POT_MAX_VALUE, POT_MAX_VALUE);
}

void PanicControls::update()
{
  if (digitalRead(PanicControls::interruptPin) == INTERRUPT_SET_STATE)
  {
    int pin = PanicControls::mcp.getLastInterruptPin();
    if (pin != PanicControls::selection)
    {
      XInput.releaseAll();
    }
    PanicControls::setXboxControlsForMechanism(pin);
    PanicControls::selection = pin;
    PanicControls::mcp.clearInterrupts();
  }
  else
  {
    XInput.releaseAll();
  }
}

void PanicControls::setXboxControlsForMechanism(PanicControls::MechanismControl mechanism)
{
  switch (mechanism)
  {
  case CORAL_FORWARD:
    XInput.setTrigger(XInputControl::TRIGGER_RIGHT, analogRead(ROLLER_POT_PIN));
    XInput.setButton(ROLLER_NEGATIVE_INDICATOR_BUTTON, false);
    break;
  case CORAL_REVERSE:
    XInput.setTrigger(XInputControl::TRIGGER_RIGHT, analogRead(ROLLER_POT_PIN));
    XInput.setButton(ROLLER_NEGATIVE_INDICATOR_BUTTON, true);
    break;
  case ALGAE_FORWARD:
    XInput.setTrigger(XInputControl::TRIGGER_LEFT, analogRead(ROLLER_POT_PIN));
    XInput.setButton(ROLLER_NEGATIVE_INDICATOR_BUTTON, false);
    break;
  case ALGAE_REVERSE:
    XInput.setTrigger(XInputControl::TRIGGER_LEFT, analogRead(ROLLER_POT_PIN));
    XInput.setButton(ROLLER_NEGATIVE_INDICATOR_BUTTON, true);
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
#include "PanicControls.h"

PanicControls::PanicControls(byte mcpI2CAddress, byte intPin)
    : m_mcpI2CAddress(mcpI2CAddress), m_interruptPin(intPin)
{
}

void PanicControls::init()
{
  if (!m_mcp.begin_I2C(m_mcpI2CAddress))
  {
    Serial.print("Failed to establish communication with the Panic Controls MCP23017 I2C device (0x");
    Serial.print(m_mcpI2CAddress, 16);
    Serial.println(")");
    while (1)
      ;
  }

  pinMode(m_interruptPin, INPUT_PULLUP);
  m_mcp.setupInterrupts(true, false, INTERRUPT_SET_STATE);

  for (int pin : PINS)
  {
    m_mcp.pinMode(pin, INPUT_PULLUP);
    m_mcp.setupInterruptPin(pin, BUTTON_PRESSED_STATE);
  }

  XInput.setTriggerRange(POT_MIN_VALUE, POT_MAX_VALUE);
  XInput.setJoystickRange(-POT_MAX_VALUE, POT_MAX_VALUE);
}

void PanicControls::update()
{
  if (digitalRead(m_interruptPin) == INTERRUPT_SET_STATE)
  {
    int pin = m_mcp.getLastInterruptPin();
    if (pin != m_selection)
    {
      resetJoysticks();
    }
    setXboxControlsForMechanism(pin);
    m_selection = pin;
    m_mcp.clearInterrupts();
  }
  else
  {
    resetJoysticks();
  }
}

void PanicControls::setXboxControlsForMechanism(PanicControls::MechanismControl mechanism)
{
  m_joysticksReset = false;
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

void PanicControls::resetJoysticks()
{
  if (m_joysticksReset)
    return;
  XInput.setTrigger(XInputControl::TRIGGER_RIGHT, 0);
  XInput.setTrigger(XInputControl::TRIGGER_LEFT, 0);
  XInput.setJoystickY(XInputControl::JOY_RIGHT, 0);
  XInput.setJoystickX(XInputControl::JOY_RIGHT, 0);
  XInput.setJoystickX(XInputControl::JOY_LEFT, 0);
  XInput.setJoystickY(XInputControl::JOY_LEFT, 0);
  XInput.release(ROLLER_NEGATIVE_INDICATOR_BUTTON);
  m_joysticksReset = true;
}
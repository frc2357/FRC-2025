#include "PanicControls.h"

PanicControls::PanicControls(byte mcpI2CAddress, byte intPin)
    : m_mcpI2CAddress(mcpI2CAddress), m_intPin(intPin)
{
}

void PanicControls::init()
{
  if (!m_mcp.begin_I2C(m_mcpI2CAddress))
  {
    Serial.println("Failed to begin communications with Panic Controls MCP");
    while (1)
      ;
  }

  pinMode(m_intPin, INPUT_PULLUP);
  // mirror INTA/B so we only need to connect to one int pin
  m_mcp.setupInterrupts(true, false, LOW);

  for (int pin = 0; pin < PIN_COUNT; pin++)
  {
    m_mcp.pinMode(pin, INPUT);
    // Enable interrupts when `pin` changes values
    m_mcp.setupInterruptPin(pin, CHANGE);
  }

  XInput.setTriggerRange(POT_MIN_VALUE, POT_MAX_VALUE);
  XInput.setJoystickRange(POT_MIN_VALUE, POT_MAX_VALUE);
}

void PanicControls::update()
{
  if (!digitalRead(m_intPin) && millis() > (DEBOUNCE_TIME_MILLIS + m_lastInterruptMillis))
  {
    // Each bit of `state` is the value of the pin at that index
    // e.g. 0b0001 means that pins 1-3 are low and pin 0 is high
    uint8_t pin = m_mcp.getLastInterruptPin();
    uint16_t state = m_mcp.getCapturedInterrupt();
    // Right shift to get `pin`s bit to index 0
    // Bitwise anding this with 1 tells us if `pin` is high or low
    bool pinValue = 1 && state >> pin;

    if (pinValue)
    {
      uint8_t mechIndex = pin / 2; // 2 for the 2 directions
      m_selectedMechanism = static_cast<PanicControls::Mechanism>(mechIndex);
      m_mechanismReversed = pin % 2;
    }
    else
    {
      m_selectedMechanism = PanicControls::Mechanism::NONE;
      m_mechanismReversed = false;
    }

    m_lastInterruptMillis = millis();
  }

  if (m_selectedMechanism != PanicControls::Mechanism::NONE)
  {
    setControllerAxes();
  }
}

void PanicControls::setControllerAxes()
{
  switch (m_selectedMechanism)
  {
  case CORAL_INTAKE_ROLLER:
    setCoralRollers(true);
    break;
  case ALGAE_INTAKE_ROLLER:
    setAlgaeRollers(true);
    break;
  case ELEVATOR:
    setElevator(true);
    break;
  case LATERATOR:
    setLaterator(true);
    break;
  case ALGAE_PIVOT:
    setAlgaePivot(true);
    break;
  case CLIMBER:
    setClimber(true);
    break;
  default:
    // Clear all axes
    setCoralRollers(false);
    setAlgaeRollers(false);
    setElevator(false);
    setLaterator(false);
    setAlgaePivot(false);
    setClimber(false);
    break;
  }
}

void PanicControls::setCoralRollers(bool on)
{
  XInput.setTrigger(XInputControl::TRIGGER_RIGHT, on * analogRead(ROLLER_POT_PIN));
  XInput.setButton(XInputControl::BUTTON_R3, on && m_mechanismReversed);
}

void PanicControls::setAlgaeRollers(bool on)
{
  XInput.setTrigger(XInputControl::TRIGGER_LEFT, on * analogRead(ROLLER_POT_PIN));
  XInput.setButton(XInputControl::BUTTON_L3, on && m_mechanismReversed);
}

void PanicControls::setElevator(bool on)
{
  uint16_t val = on ? analogRead(MOVEMENT_POT_PIN) : 0;
  XInput.setJoystickY(XInputControl::JOY_RIGHT, val, on && m_mechanismReversed);
}

void PanicControls::setLaterator(bool on)
{
  uint16_t val = on ? analogRead(MOVEMENT_POT_PIN) : 0;
  XInput.setJoystickX(XInputControl::JOY_RIGHT, val, on && m_mechanismReversed);
}

void PanicControls::setAlgaePivot(bool on)
{
  uint16_t val = on ? analogRead(MOVEMENT_POT_PIN) : 0;
  XInput.setJoystickX(XInputControl::JOY_LEFT, val, on && m_mechanismReversed);
}

void PanicControls::setClimber(bool on)
{
  uint16_t val = on ? analogRead(MOVEMENT_POT_PIN) : 0;
  XInput.setJoystickY(XInputControl::JOY_LEFT, val, on && m_mechanismReversed);
}
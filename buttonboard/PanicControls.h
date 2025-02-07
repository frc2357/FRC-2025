#ifndef PANIC_CONTROLS_H
#define PANIC_CONTROLS_H

#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
#include <XInput.h>

#define DEBOUNCE_TIME_MILLIS 50

#define PIN_COUNT 12

#define ROLLER_MECHANISM_SLIDER_FORWARD_PIN A0
#define ROLLER_MECHANISM_SLIDER_REVERSE_PIN A1
#define MOVEMENT_MECHANISM_SLIDER_FORWARD_PIN A2
#define MOVEMENT_MECHANISM_SLIDER_REVERSE_PIN A3

#define POT_MIN_VALUE 0
#define POT_MAX_VALUE 1023

#define ROLLER_POT_PIN -1
#define MOVEMENT_POT_PIN -1

/**
 * ! SWITCH WIRING
 * Mechanism selection buttons will be wired from pins 0 to 11
 * These will be in order base on the PanicControls::Mechanism enum
 * pin0 will be forward for mech0, pin1 will be reverse for mech0,
 * pin2 will be forward for mech1, etc.
 * This is crucial to efficiently determining which mechanism was selected
 */

/**
 * ! MECHANISM TO AXIS MAPPING
 * Mechanisms will map to each mech. XInput doesn't make it easy to do this with a simple c++ map
 * CORAL_INTAKE_ROLLER = TRIGGER_RIGHT + BUTTON_R3 (for reverse)
 * ALGAE_INTAKE_ROLLER = TRIGGER_LEFT + BUTTON_L3 (for reverse)
 * ELEVATOR = JOY_RIGHT Y
 * LATERATOR = JOY_RIGHT X
 * ALGAE_PIVOT = JOY_LEFT X
 * CLIMBER = JOY_LEFT Y
 */

class PanicControls
{
public:
    enum Mechanism
    {
        NONE = -1,
        // Roller mechanisms
        CORAL_INTAKE_ROLLER = 0,
        ALGAE_INTAKE_ROLLER = 1,
        // Movement mechanisms
        ELEVATOR = 2,
        LATERATOR = 3,
        ALGAE_PIVOT = 4,
        CLIMBER = 5
    };

    PanicControls(byte mcpI2CAddress, byte intPin);
    void init();
    void update();
    void setControllerAxes();

private:
    static void setCoralRollers(bool on);
    static void setAlgaeRollers(bool on);
    static void setElevator(bool on);
    static void setLaterator(bool on);
    static void setAlgaePivot(bool on);
    static void setClimber(bool on);
    // I2C address of the MCP23017
    byte m_mcpI2CAddress;
    // Arduino pin that is connected to the MCP INTA/B pin
    byte m_intPin;
    Adafruit_MCP23X17 m_mcp;

    PanicControls::Mechanism m_selectedMechanism;
    bool m_mechanismReversed;

    long m_lastInterruptMillis;
};

#endif // PANIC_CONTROLS_H
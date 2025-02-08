#include "PanicControls.h"
#include "ScoringSelection.h"

#define LEFT_SCORING_LEVEL_SELECTOR_ADDRESS 0x30
#define RIGHT_SCORING_LEVEL_SELECTOR_ADDRESS 0x31

#define PANIC_CONTROLS_MCP_I2C_ADDRESS 0x20
#define PANIC_CONTROLS_MCP_INTERRUPT_PIN 7 // TODO: Set this

void setup()
{
    XInput.begin();

    PanicControls::init(PANIC_CONTROLS_MCP_I2C_ADDRESS, PANIC_CONTROLS_MCP_INTERRUPT_PIN);
}

void loop()
{
}
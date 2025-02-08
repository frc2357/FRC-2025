#include "PanicControls.h"

#define PANIC_CONTROLS_MCP_I2C_ADDRESS 0x00 // TODO: Set this

#define PANIC_CONTROLS_MCP_INTERRUPT_PIN 7 // TODO: Set this

PanicControls panicControls(PANIC_CONTROLS_MCP_I2C_ADDRESS, PANIC_CONTROLS_MCP_INTERRUPT_PIN);

void setup()
{
    XInput.begin();

    panicControls.init();
}

void loop()
{
    panicControls.update();
}
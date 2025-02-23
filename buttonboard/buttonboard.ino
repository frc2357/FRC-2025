#include "LevelSelection.h"
#include "BranchSelection.h"
#include "PanicControls.h"

#define LEFT_LEVEL_KEYPAD_ADDRESS 0x30
#define RIGHT_LEVEL_KEYPAD_ADDRESS 0x31

#define PANIC_CONTROLS_MCP_I2C_ADDRESS 0x20
#define PANIC_CONTROLS_MCP_INT_PIN 1

LevelSelection level(LEFT_LEVEL_KEYPAD_ADDRESS, RIGHT_LEVEL_KEYPAD_ADDRESS);
BranchSelection branch;
PanicControls panic(PANIC_CONTROLS_MCP_I2C_ADDRESS, PANIC_CONTROLS_MCP_INT_PIN);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  XInput.begin();

  level.init();
  branch.init();
  panic.init();
}

void loop()
{
  level.update();
  branch.update();
  panic.update();
}

// Reef branch selection debouncer methods
void onPinActivated(int pin)
{
  branch.onPinActivated(pin);
}

void onPinDeactivated(int pin)
{
  branch.onPinDeactivated(pin);
}
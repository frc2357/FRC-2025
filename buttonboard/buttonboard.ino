#include "LevelSelection.h"
#include "BranchSelection.h"
#include "PanicControls.h"

#define LEFT_LEVEL_KEYPAD_ADDRESS 0x30
#define RIGHT_LEVEL_KEYPAD_ADDRESS 0x31

LevelSelection level(LEFT_LEVEL_KEYPAD_ADDRESS, RIGHT_LEVEL_KEYPAD_ADDRESS);
BranchSelection branch;

#define PANIC_CONTROLS_MCP_I2C_ADDRESS 0x20
#define PANIC_CONTROLS_MCP_INT_PIN 1

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  XInput.begin();

  level.init();
  branch.init();
  PanicControls::init(PANIC_CONTROLS_MCP_I2C_ADDRESS, PANIC_CONTROLS_MCP_INT_PIN);
}

void loop()
{
  level.update();
  branch.update();
  PanicControls::update();
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
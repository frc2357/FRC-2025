#include "LevelSelection.h"
#include "BranchSelection.h"
#include "PanicControls.h"
#include "Leds.h"

#define LEFT_LEVEL_KEYPAD_ADDRESS 0x30
#define RIGHT_LEVEL_KEYPAD_ADDRESS 0x31

#define PANIC_CONTROLS_MCP_I2C_ADDRESS 0x20
#define PANIC_CONTROLS_MCP_INT_PIN 1

#define LEDS_MCP_I2C_ADDRESS 0x20

// LevelSelection level(LEFT_LEVEL_KEYPAD_ADDRESS, RIGHT_LEVEL_KEYPAD_ADDRESS);
// BranchSelection branch;
// PanicControls panic(PANIC_CONTROLS_MCP_I2C_ADDRESS, PANIC_CONTROLS_MCP_INT_PIN);
Leds leds(LEDS_MCP_I2C_ADDRESS);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  XInput.begin();

  // level.init();
  // branch.init();
  // panic.init();
  leds.init();
}

void loop()
{
  // level.update();
  // branch.update();
  // panic.update();
  delay(100);
  // leds.update(level.getSelection(), branch.getSelection());
  leds.update(LevelSelection::Level::L2, BranchSelection::Branch::A);
}

// Reef branch selection debouncer methods
void onPinActivated(int pin)
{
  // branch.onPinActivated(pin);
}

void onPinDeactivated(int pin)
{
  // branch.onPinDeactivated(pin);
}
#include "LevelSelection.h"
#include "BranchSelection.h"

#define LEFT_LEVEL_KEYPAD_ADDRESS 0x30
#define RIGHT_LEVEL_KEYPAD_ADDRESS 0x31

LevelSelection level(LEFT_LEVEL_KEYPAD_ADDRESS, RIGHT_LEVEL_KEYPAD_ADDRESS);
BranchSelection branch;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  XInput.begin();

  level.init();
  branch.init();
}

void loop()
{
  level.update();
  branch.update();
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
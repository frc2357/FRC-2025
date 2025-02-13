#include "LevelSelection.h"

#define LEFT_LEVEL_KEYPAD_ADDRESS 0x30
#define RIGHT_LEVEL_KEYPAD_ADDRESS 0x31
#define LEVEL_KEYPAD_INTERRUPT_PIN 0

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  XInput.begin();

  LevelSelection::init(LEFT_LEVEL_KEYPAD_ADDRESS, RIGHT_LEVEL_KEYPAD_ADDRESS, LEVEL_KEYPAD_INTERRUPT_PIN);
}

void loop()
{
  // LevelSelection updates are handled with interrupts so nothing needs to happen here
}
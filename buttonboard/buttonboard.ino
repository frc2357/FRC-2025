#include "LevelSelection.h"

#define LEFT_LEVEL_KEYPAD_ADDRESS 0x30
#define RIGHT_LEVEL_KEYPAD_ADDRESS 0x31

LevelSelection level(LEFT_LEVEL_KEYPAD_ADDRESS, RIGHT_LEVEL_KEYPAD_ADDRESS);

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  XInput.begin();

  level.init();
}

void loop()
{
  level.update();
}
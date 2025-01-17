#include <Joystick.h>

// Driver station shows 16 buttons by default
// If controller has more, it shows 21
// _Joystick class defaults to 32 which can be read by driver station just isn't visually displayed properly
#define NUM_JOYSTICK_BUTTONS 32

Joystick_ joystick;
int currentButton = 0;

void setup()
{
  Serial.begin(115200);
  joystick.begin();
}

void loop()
{
  Serial.println(currentButton);

  joystick.setButton(currentButton, 1);
  delay(200);
  joystick.setButton(currentButton, 0);

  currentButton++;
  if (currentButton == NUM_JOYSTICK_BUTTONS)
  {
    currentButton = 0;
  }
}
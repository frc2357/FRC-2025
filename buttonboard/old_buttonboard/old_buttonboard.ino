/*
 * NOTE: In Adafruit_Keypad_Ringbuffer.h, I have updated the typedef for Adafruit_Keypad_Ringbuffer to be
 * Adafruit_Keypad_RingbufferN<256> Due to issues I was facing when using the library
 */

#include <Arduino.h>
#include <Adafruit_Keypad.h>
#include <XInput.h>
#include <Adafruit_NeoPixel.h>

#define LOOP_DELAY_MS 10

#define ROWS 3
#define COLS 6

#define REEF_SIDE_ROW 0
#define SCORING_LOCATION_ROWS 4

#define LED_PIN A0
#define LED_COUNT 27

#define COLOR_ON 0xFFFFFFFF
#define COLOR_OFF 0x00000000

uint8_t rowPins[ROWS] = {4, 3, 2};
uint8_t colPins[COLS] = {6, 7, 8, 9, 10, 11};
char keys[ROWS][COLS] = {
    {'a', 'b', 'c', 'd', 'e', 'f'},
    {'g', 'h', 'i', 'j', 'k', 'l'},
    {'m', 'n', 'o', 'p', 'q', 'r'}};

XInputControl reefSideButtons[6] = {
    BUTTON_A,
    BUTTON_B,
    BUTTON_X,
    BUTTON_Y,
    BUTTON_BACK,
    BUTTON_START,
};
XInputControl scoringLevelButtons[4] = {
    DPAD_UP,
    DPAD_RIGHT,
    DPAD_DOWN,
    DPAD_LEFT,
};
XInputControl scoringDirectionButtons[2] = {
    BUTTON_LB,
    BUTTON_RB,
};

Adafruit_Keypad keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
Adafruit_NeoPixel ledStrip(LED_COUNT, LED_PIN, NEO_GRB);

int selectedReefSide = -1;
int selectedScoringLevel = -1;
int selectedScoringDirection = -1;

void setup()
{
  XInput.begin();

  keypad.begin();

  ledStrip.begin();
  ledStrip.setBrightness(40);
  ledStrip.show();
}

void loop()
{
  keypad.tick();
  scanKeypad();
  delay(LOOP_DELAY_MS);
}

void scanKeypad()
{
  // Signal that we've scored and to deselect everything
  if (XInput.getRumble())
  {
    if (selectedScoringDirection != -1)
      XInput.release(scoringDirectionButtons[selectedScoringDirection - 1]); // Subtract 1 because row 0 is reefside
    if (selectedScoringLevel != -1)
      XInput.release(scoringLevelButtons[selectedScoringLevel]);
    if (selectedReefSide != -1)
      XInput.release(reefSideButtons[selectedReefSide]);

    ledStrip.setPixelColor(getLEDIndex(REEF_SIDE_ROW, selectedReefSide), COLOR_OFF);
    ledStrip.setPixelColor(getLEDIndex(selectedScoringDirection, selectedScoringLevel), COLOR_OFF);
    ledStrip.show();

    selectedReefSide = -1;
    selectedScoringLevel = -1;
    selectedScoringDirection = -1;
  }
  while (keypad.available())
  {
    keypadEvent e = keypad.read();
    uint8_t row = e.bit.ROW;
    uint8_t col = e.bit.COL;

    switch (e.bit.EVENT)
    {
    case KEY_JUST_PRESSED:
      onKeyPress(row, col);
      break;
    case KEY_JUST_RELEASED:
      onKeyRelease(row, col);
      break;
    default:
      break;
    }
  }
}

void onKeyPress(int row, int col)
{
  if (row == REEF_SIDE_ROW)
  {
    setReefSide(row, col);
  }
  else
  {
    setScoringLocation(row, col);
  }
  ledStrip.show();
}

void onKeyRelease(int row, int col) {}

void setReefSide(int row, int col)
{
  bool select = selectedReefSide != col;
  uint32_t color = select ? COLOR_ON : COLOR_OFF;

  if (select && selectedReefSide != -1)
  {
    XInput.release(reefSideButtons[selectedReefSide]);
    ledStrip.setPixelColor(getLEDIndex(REEF_SIDE_ROW, selectedReefSide), COLOR_OFF);
  }

  selectedReefSide = select ? col : -1;
  XInput.setButton(reefSideButtons[col], select);
  ledStrip.setPixelColor(getLEDIndex(row, col), color);
}

void setScoringLocation(int row, int col)
{
  if (col >= SCORING_LOCATION_ROWS)
    return; // We only have SCORING_LOCATION_ROWS rows for the scoring location buttons
  bool select = (selectedScoringLevel != col) || selectedScoringDirection != row;
  uint32_t color = select ? COLOR_ON : COLOR_OFF;

  if (select && selectedScoringLevel != -1 && selectedScoringDirection != -1)
  {
    XInput.release(scoringLevelButtons[selectedScoringLevel]);
    XInput.release(scoringDirectionButtons[selectedScoringDirection - 1]);
    ledStrip.setPixelColor(getLEDIndex(selectedScoringDirection, selectedScoringLevel), COLOR_OFF);
  }

  selectedScoringLevel = select ? col : -1;
  selectedScoringDirection = select ? row : -1;
  XInput.setButton(scoringLevelButtons[col], select);
  XInput.setButton(scoringDirectionButtons[row - 1], select);
  ledStrip.setPixelColor(getLEDIndex(row, col), color);
}

size_t getLEDIndex(uint8_t row, uint8_t col)
{
  size_t index;
  uint8_t buttonRow = abs(row + 1 - ROWS);
  uint8_t buttonCol = col;

  if (buttonRow % 2 == 0)
  { // even row
    index = buttonRow * COLS + buttonCol;
  }
  else
  { // odd row the neopixels go BACKWARDS!
    index = buttonRow * COLS + ((COLS - 1) - buttonCol);
  }
  if (buttonRow != 0)
  {
    index += 6; // Temporary while we still have 3x9 grid
  }

  return index;
}
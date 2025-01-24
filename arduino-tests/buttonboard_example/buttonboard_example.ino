#include <Arduino.h>
#include <Adafruit_Keypad.h>
#include <Joystick.h>
#include <Adafruit_NeoPixel.h>

#define LOOP_DELAY_MS 10

#define ROWS 3
#define COLS 6

#define REEF_SIDE_ROW 0

#define LED_PIN A0
#define LED_COUNT 14

#define COLOR_ON 0xFFFFFFFF
#define COLOR_OFF 0x00000000

uint8_t rowPins[ROWS] = {4, 3, 2};
uint8_t colPins[COLS] = {6, 7, 8, 9, 10, 11};
char keys[ROWS][COLS] = {
  {'a', 'b', 'c', 'd', 'e', 'f'},
  {'g', 'h', 'i', 'j', 'k', 'l'},
  {'m', 'n', 'o', 'p', 'q', 'r'}
};

uint8_t reefsideKeys[6] = { 0, 1, 2, 3, 4, 5 };
uint8_t scoringLevelKeys[4] = { 6, 7, 8, 9 };
uint8_t scoringDirectionKeys[2] = { 10, 11 };

Adafruit_Keypad keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
Adafruit_NeoPixel ledStrip(LED_COUNT, LED_PIN, NEO_GRB);
Joystick_ joystick(0x03, 0x04, 14, 0, false, false, false, false, false, false, false, false, false, false, false);

int selectedReefSide = -1;
int selectedScoringLevel = -1;
int selectedScoringDirection = -1;

void setup() {
  joystick.begin();

  keypad.begin();

  ledStrip.begin();
  ledStrip.setBrightness(40);
  ledStrip.show();
}

void loop() {
  keypad.tick();
  scanKeypad();
  delay(LOOP_DELAY_MS);
}

void scanKeypad() {
  while (keypad.available()) {
    keypadEvent e = keypad.read();
    uint8_t row = e.bit.ROW;
    uint8_t col = e.bit.COL;

    switch (e.bit.EVENT) {
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

void onKeyPress(int row, int col) {
  if (row == REEF_SIDE_ROW) {
    setReefSide(row, col);
  } else {
    setScoringLocation(row, col);
  }
  ledStrip.show();
}

void onKeyRelease(int row, int col) {
}

void setReefSide(int row, int col) {
  bool select = selectedReefSide != col;
  uint32_t color = select ? COLOR_ON : COLOR_OFF;

  if (select) {
    joystick.setButton(reefsideKeys[selectedReefSide], 0);
    ledStrip.setPixelColor(getLEDIndex(row, col), COLOR_OFF);
  }

  selectedReefSide = select ? col : -1;
  joystick.setButton(reefsideKeys[col], select);
  ledStrip.setPixelColor(getLEDIndex(row, col), color);
}

void setScoringLocation(int row, int col) {
  bool select = selectedScoringLevel != col || selectedScoringDirection != row;
  uint32_t color = select ? COLOR_ON : COLOR_OFF;

  if (select) {
    joystick.setButton(scoringLevelKeys[selectedScoringLevel], 0);
    joystick.setButton(scoringDirectionKeys[selectedScoringDirection - 1], 0);
    ledStrip.setPixelColor(getLEDIndex(selectedScoringDirection, selectedScoringLevel), COLOR_OFF);
  }

  selectedScoringLevel = select ? col : -1;
  selectedScoringDirection = select ? row : -1;
  joystick.setButton(scoringLevelKeys[col], select);
  joystick.setButton(scoringDirectionKeys[row - 1], select);
  ledStrip.setPixelColor(getLEDIndex(row, col), color);
}

size_t getLEDIndex(uint8_t row, uint8_t col) {
    size_t index; 
    uint8_t buttonRow = abs(row + 1 - ROWS);
    uint8_t buttonCol = col;

    if (buttonRow % 2 == 0) { // even row
      index = buttonRow * COLS + buttonCol;
    } else { // odd row the neopixels go BACKWARDS!
      index = buttonRow * COLS + ((COLS - 1) - buttonCol);
    }
    if (buttonRow != 0) {
      index += 6; // Temporary while we still have 3x9 grid
    }

    return index;
  }
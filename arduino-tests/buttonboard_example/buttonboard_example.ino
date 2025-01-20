#include <Arduino.h>
#include <Adafruit_Keypad.h>
#include <Joystick.h>
#include <Adafruit_NeoPixel.h>

#define LOOP_DELAY_MS 10

#define GRID_ROWS 6
#define GRID_COLS 3

#define REEF_SIDE_COL 0

#define LED_PIN 0
#define LED_COUNT 14

#define COLOR_ON 0xFFFFFFFF
#define COLOR_OFF 0x00000000

uint8_t rowPins[GRID_ROWS];
uint8_t colPins[GRID_COLS];
uint8_t reefsideKeys[6] = {0, 1, 2, 3, 4, 5};
uint8_t scoringLevelKeys[4] = {6, 7, 8, 9};
uint8_t scoringDirectionKeys[2] = {10, 11};
// Not sure how necessary it is to fill this with values
uint8_t keys[GRID_ROWS][GRID_COLS];

Adafruit_Keypad keypad(makeKeymap(keys), rowPins, colPins, GRID_ROWS, GRID_COLS);
Adafruit_NeoPixel ledStrip(LED_COUNT, LED_PIN, NEO_GRB);
Joystick_ joystick;

int selectedReefSide, selectedScoringLevel, selectedScoringDirection;

void setup()
{
    joystick.begin();
    ledStrip.begin();
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

void onKeyPress(uint8_t row, uint8_t col)
{
    if (col == REEF_SIDE_COL)
    {
        setReefSide(row, col);
    }
    else
    {
        setScoringLocation(row, col);
    }
    ledStrip.show();
}

void onKeyRelease(uint8_t row, uint8_t col)
{
}

void setReefSide(uint8_t row, uint8_t col)
{
    bool select = selectedReefSide == row;
    uint32_t color = select ? COLOR_ON : COLOR_OFF;

    selectedReefSide = select ? row : -1;
    joystick.setButton(reefsideKeys[row], select);
    ledStrip.setPixelColor(getLEDIndex(row, col), color);
}

void setScoringLocation(uint8_t row, uint8_t col)
{
    bool select = selectedScoringLevel == row && selectedScoringDirection == col;
    uint32_t color = select ? COLOR_ON : COLOR_OFF;

    selectedScoringLevel = select ? row : -1;
    selectedScoringDirection = select ? col : -1;
    joystick.setButton(scoringLevelKeys[row], select);
    joystick.setButton(scoringDirectionKeys[col - 1], select);
    ledStrip.setPixelColor(getLEDIndex(row, col), color);
}

uint8_t getLEDIndex(uint8_t row, uint8_t col)
{
    return (col * GRID_COLS) + row;
}
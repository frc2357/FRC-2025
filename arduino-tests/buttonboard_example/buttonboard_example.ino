#include <Arduino.h>
#include <Adafruit_Keypad.h>
#include <Joystick.h>
#include <Adafruit_Neopixel.h>

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
uint8_t keys[GRID_ROWS][GRID_COLS] = {
    {0, 6, 12},
    {1, 7, 13},
    {2, 8, 14},
    {3, 9, 15},
    {4},
    {5},
};

Adafruit_Keypad keypad(makeKeymap(keys), rowPins, colPins, GRID_ROWS, GRID_COLS);
Adafruit_Neopixel ledStrip(LED_COUNT, LED_PIN, NEO_GRB);
Joystick_ joystick;

int selectedSideRow, selectedSideCol, selectedLevelRow, selectedLevelCol;

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
        setScoringLevel(row, col);
    }
    ledStrip.show();
}

void onKeyRelease(uint8_t row, uint8_t col)
{
}

void setReefSide(uint8_t row, uint8_t col)
{
    bool select = selectedSideRow == row && selectedSideCol == col;
    uint32_t color = select ? COLOR_ON : COLOR_OFF;

    selectedSideRow = select ? row : -1;
    selectedSideCol = select ? col : -1;
    joystick.setButton(keys[row][col], select);
    ledStrip.setPixelColor(getLEDIndex(row, col), color);
}

void setScoringLevel(uint8_t row, uint8_t col)
{
    bool select = selectedLevelRow == row && selectedLevelCol == col;
    uint32_t color = select ? COLOR_ON : COLOR_OFF;

    selectedLevelRow = select ? row : -1;
    selectedLevelCol = select ? col : -1;
    joystick.setButton(keys[row][col], select);
    ledStrip.setPixelColor(getLEDIndex(row, col), color);
}

uint8_t getLEDIndex(uint8_t row, uint8_t col)
{
    return (col * GRID_COLS) + row;
}
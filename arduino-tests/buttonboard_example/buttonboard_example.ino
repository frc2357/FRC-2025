#include <Arduino.h>
#include <Adafruit_Keypad.h>
#include <Joystick.h>

#define LOOP_DELAY_MS 10

#define GRID_ROWS 6
#define GRID_COLS 3

#define REEF_SIDE_COL 0

uint8_t rowPins[GRID_ROWS];
uint8_t colPins[GRID_COLS];
uint8_t keys[GRID_ROWS][GRID_COLS] = {
    {0, 6, 10},
    {1, 7, 11},
    {2, 8, 12},
    {3, 9, 13},
    {4},
    {5},
};

Adafruit_Keypad keypad(makeKeymap(keys), rowPins, colPins, GRID_ROWS, GRID_COLS);
Joystick_ joystick;

int selectedSideRow, selectedSideCol, selectedLevelRow, selectedLevelCol;

void setup()
{
    joystick.begin();
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
    if (selectedSideRow == row && selectedSideCol == col)
    {
        joystick.setButton(keys[row][col], 0);
        selectedSideRow = -1;
        selectedSideCol = -1;
        return;
    }
    if (selectedLevelRow == row && selectedLevelCol == col)
    {
        joystick.setButton(keys[row][col], 0);
        selectedLevelRow = -1;
        selectedLevelCol = -1;
        return;
    }

    if (col == REEF_SIDE_COL)
    {
        selectedSideRow = row;
        selectedSideCol = col;
        joystick.setButton(keys[row][col], 1);
    }
    else
    {
        selectedLevelRow = row;
        selectedLevelCol = col;
        joystick.setButton(keys[row][col], 1);
    }
}

void onKeyRelease(uint8_t row, uint8_t col)
{
}
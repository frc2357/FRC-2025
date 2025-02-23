#ifndef BRANCH_SELECTION_H
#define BRANCH_SELECTION_H

#include <Arduino.h>
#include "FTDebouncer.h"
#include <XInput.h>

#define NUM_BUTTONS 12

#define PIN_DEBOUNCE_TIME_MILLIS 30

#define REEF_DIR_LEFT XInputControl::BUTTON_LB
#define REEF_DIR_RIGHT XInputControl::BUTTON_RB

#define REEF_SIDE1 XInputControl::BUTTON_A
#define REEF_SIDE2 XInputControl::BUTTON_B
#define REEF_SIDE3 XInputControl::BUTTON_X
#define REEF_SIDE4 XInputControl::BUTTON_Y
#define REEF_SIDE5 XInputControl::BUTTON_BACK
#define REEF_SIDE6 XInputControl::BUTTON_START

class BranchSelection
{
public:
    // integer values correspond to pin numbers
    enum Branch
    {
        NONE = -1,
        A = 4,
        B = 5,
        C = 6,
        D = 7,
        E = 8,
        F = 9,
        G = 10,
        H = 11,
        I = 12,
        J = 13,
        K = 14,
        L = 16,
    };
    static int PINS[NUM_BUTTONS];

    BranchSelection();

    void init();
    void update();

    BranchSelection::Branch getSelection();

    void onPinActivated(int pin);
    void onPinDeactivated(int pin);

private:
    void setXboxButtonsForBranch(BranchSelection::Branch selection, bool selected);

    FTDebouncer m_debouncer;
    BranchSelection::Branch m_selection;
};

#endif // BRANCH_SELECTION_H
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
        A = 19,
        B = 13,
        C = 12,
        D = 18,
        E = 5,
        F = 4,
        G = 6,
        H = 7,
        I = 11,
        J = 9,
        K = 10,
        L = 8,
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
    BranchSelection::Branch m_selection = BranchSelection::Branch::NONE;
};

#endif // BRANCH_SELECTION_H
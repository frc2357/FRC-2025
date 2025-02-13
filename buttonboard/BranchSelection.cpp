#include "BranchSelection.h"

int BranchSelection::pins[NUM_BUTTONS] = {
    BranchSelection::Branch::A,
    BranchSelection::Branch::B,
    BranchSelection::Branch::C,
    BranchSelection::Branch::D,
    BranchSelection::Branch::E,
    BranchSelection::Branch::F,
    BranchSelection::Branch::G,
    BranchSelection::Branch::H,
    BranchSelection::Branch::I,
    BranchSelection::Branch::J,
    BranchSelection::Branch::K,
    BranchSelection::Branch::L,
};

FTDebouncer BranchSelection::debouncer(PIN_DEBOUNCE_TIME_MILLIS);

BranchSelection::Branch BranchSelection::selection;

void BranchSelection::init()
{
    for (int pin : BranchSelection::pins)
    {
        BranchSelection::debouncer.addPin(pin, HIGH, BranchSelection::onPinActivated, BranchSelection::onPinDeactivated, INPUT_PULLUP);
    }
    BranchSelection::debouncer.begin();
}

BranchSelection::Branch BranchSelection::getSelection()
{
    return BranchSelection::selection;
}

void BranchSelection::onPinActivated(int pin)
{
    BranchSelection::Branch branch = static_cast<BranchSelection::Branch>(pin);
    XInputControl *buttons = BranchSelection::getXboxButtonsFromBranch(branch);
    if (branch == BranchSelection::selection)
    {
        BranchSelection::selection = BranchSelection::Branch::NONE;
        for (int i = 0; i < 2; i++)
        {
            XInput.release(static_cast<XInputControl>(i));
        }
    }
    else
    {
        BranchSelection::selection = branch;
        for (int i = 0; i < 2; i++)
        {
            XInput.press(static_cast<XInputControl>(i));
        }
    }
}

void BranchSelection::onPinDeactivated(int pin)
{
}

XInputControl *BranchSelection::getXboxButtonsFromBranch(BranchSelection::Branch branch)
{
    switch (branch)
    {
    case A:
        return new XInputControl[2]{REEF_DIR_LEFT, REEF_SIDE1};
    case B:
        return new XInputControl[2]{REEF_DIR_RIGHT, REEF_SIDE1};
    case C:
        return new XInputControl[2]{REEF_DIR_LEFT, REEF_SIDE2};
    case D:
        return new XInputControl[2]{REEF_DIR_RIGHT, REEF_SIDE2};
    case E:
        return new XInputControl[2]{REEF_DIR_LEFT, REEF_SIDE3};
    case F:
        return new XInputControl[2]{REEF_DIR_RIGHT, REEF_SIDE3};
    case G:
        return new XInputControl[2]{REEF_DIR_LEFT, REEF_SIDE4};
    case H:
        return new XInputControl[2]{REEF_DIR_RIGHT, REEF_SIDE4};
    case I:
        return new XInputControl[2]{REEF_DIR_LEFT, REEF_SIDE5};
    case J:
        return new XInputControl[2]{REEF_DIR_RIGHT, REEF_SIDE5};
    case K:
        return new XInputControl[2]{REEF_DIR_LEFT, REEF_SIDE6};
    case L:
        return new XInputControl[2]{REEF_DIR_RIGHT, REEF_SIDE6};
    }
}
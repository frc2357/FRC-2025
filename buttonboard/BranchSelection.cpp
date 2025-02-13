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

    if (branch == BranchSelection::selection)
    {
        // Deselect already selected branch
        BranchSelection::setXboxButtonsForBranch(BranchSelection::selection, false);
        BranchSelection::selection = BranchSelection::Branch::NONE;
    }
    else
    {
        BranchSelection::setXboxButtonsForBranch(BranchSelection::selection, false);
        BranchSelection::setXboxButtonsForBranch(branch, true);
        BranchSelection::selection = branch;
    }
}

void BranchSelection::onPinDeactivated(int pin)
{
}

XInputControl *BranchSelection::setXboxButtonsForBranch(BranchSelection::Branch branch, bool selected)
{
    switch (branch)
    {
    case A:
        XInput.setButton(REEF_DIR_LEFT, selected);
        XInput.setButton(REEF_SIDE1, selected);
        break;
    case B:
        XInput.setButton(REEF_DIR_RIGHT, selected);
        XInput.setButton(REEF_SIDE1, selected);
        break;
    case C:
        XInput.setButton(REEF_DIR_LEFT, selected);
        XInput.setButton(REEF_SIDE2, selected);
        break;
    case D:
        XInput.setButton(REEF_DIR_RIGHT, selected);
        XInput.setButton(REEF_SIDE2, selected);
        break;
    case E:
        XInput.setButton(REEF_DIR_LEFT, selected);
        XInput.setButton(REEF_SIDE3, selected);
        break;
    case F:
        XInput.setButton(REEF_DIR_RIGHT, selected);
        XInput.setButton(REEF_SIDE3, selected);
        break;
    case G:
        XInput.setButton(REEF_DIR_LEFT, selected);
        XInput.setButton(REEF_SIDE4, selected);
        break;
    case H:
        XInput.setButton(REEF_DIR_RIGHT, selected);
        XInput.setButton(REEF_SIDE4, selected);
        break;
    case I:
        XInput.setButton(REEF_DIR_LEFT, selected);
        XInput.setButton(REEF_SIDE5, selected);
        break;
    case J:
        XInput.setButton(REEF_DIR_RIGHT, selected);
        XInput.setButton(REEF_SIDE5, selected);
        break;
    case K:
        XInput.setButton(REEF_DIR_LEFT, selected);
        XInput.setButton(REEF_SIDE6, selected);
        break;
    case L:
        XInput.setButton(REEF_DIR_RIGHT, selected);
        XInput.setButton(REEF_SIDE6, selected);
        break;
    }
}
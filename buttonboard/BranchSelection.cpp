#include "BranchSelection.h"

int BranchSelection::PINS[NUM_BUTTONS] = {
    Branch::A,
    Branch::B,
    Branch::C,
    Branch::D,
    Branch::E,
    Branch::F,
    Branch::G,
    Branch::H,
    Branch::I,
    Branch::J,
    Branch::K,
    Branch::L,
};

BranchSelection::BranchSelection() : m_debouncer(PIN_DEBOUNCE_TIME_MILLIS)
{
}

void BranchSelection::init()
{
    for (int pin : BranchSelection::PINS)
    {
        m_debouncer.addPin(pin, HIGH, INPUT_PULLUP);
    }
    m_debouncer.begin();
}

void BranchSelection::update()
{
    m_debouncer.update();
}

BranchSelection::Branch BranchSelection::getSelection()
{
    return m_selection;
}

void BranchSelection::onPinActivated(int pin)
{
    Branch branch = static_cast<Branch>(pin);

    // Deselect already selected branch
    setXboxButtonsForBranch(m_selection, false);

    if (branch == m_selection)
    {
        m_selection = Branch::NONE;
    }
    else
    {
        setXboxButtonsForBranch(branch, true);
        m_selection = branch;
    }
}

void BranchSelection::onPinDeactivated(int pin)
{
}

void BranchSelection::setXboxButtonsForBranch(BranchSelection::Branch branch, bool selected)
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
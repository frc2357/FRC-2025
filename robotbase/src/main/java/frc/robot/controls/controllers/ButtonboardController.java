package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.GenericHID;

public class ButtonboardController extends GenericHID {
    public enum ReefSide {
        kA(0),
        kB(1),
        kC(2),
        kD(3),
        kE(4),
        kF(5);

        public final int buttonValue;

        ReefSide(int buttonValue) {
            this.buttonValue = buttonValue;
        }
    }

    public enum ScoringLevel {
        kL4(6),
        kL3(7),
        kL2(8),
        kL1(9);

        public final int buttonValue;

        ScoringLevel(int buttonValue) {
            this.buttonValue = buttonValue;
        }
    }

    public enum ScoringDirection {
        kRight(10),
        kLeft(11);

        public final int buttonValue;

        ScoringDirection(int buttonValue) {
            this.buttonValue = buttonValue;
        }
    }

    public ButtonboardController(final int port) {
        super(port);
    }

    public ReefSide getSelectedReefSide() {
        for (ReefSide side : ReefSide.values()) {
            if (getRawButton(side.buttonValue)) {
                return side;
            }
        }
        return null;
    }

    public ScoringLevel getSelectedScoringLevel() {
        for (ScoringLevel lvl : ScoringLevel.values()) {
            if (getRawButton(lvl.buttonValue)) {
                return lvl;
            }
        }
        return null;
    }

    public ScoringDirection getSelectedScoringDirection() {
        for (ScoringDirection dir : ScoringDirection.values()) {
            if (getRawButton(dir.buttonValue)) {
                return dir;
            }
        }
        return null;
    }
}

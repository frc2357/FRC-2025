package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.GenericHID;

public class ButtonboardController extends GenericHID {
    public enum ReefSide {
        kNone(0),
        kA(1),
        kB(2),
        kC(3),
        kD(4),
        kE(5),
        kF(6);

        public final int buttonValue;

        ReefSide(int buttonValue) {
            this.buttonValue = buttonValue;
        }
    }

    public enum ScoringLevel {
        kNone(0),
        kL4(7),
        kL3(8),
        kL2(9),
        kL1(10);

        public final int buttonValue;

        ScoringLevel(int buttonValue) {
            this.buttonValue = buttonValue;
        }
    }

    public enum ScoringDirection {
        kNone(0),
        kLeft(11),
        kRight(12);

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
            if (side.buttonValue != 0 && getRawButton(side.buttonValue)) {
                return side;
            }
        }
        return ReefSide.kNone;
    }

    public ScoringLevel getSelectedScoringLevel() {
        for (ScoringLevel lvl : ScoringLevel.values()) {
            if (lvl.buttonValue != 0 && getRawButton(lvl.buttonValue)) {
                return lvl;
            }
        }
        return ScoringLevel.kNone;
    }

    public ScoringDirection getSelectedScoringDirection() {
        for (ScoringDirection dir : ScoringDirection.values()) {
            if (dir.buttonValue != 0 && getRawButton(dir.buttonValue)) {
                return dir;
            }
        }
        return ScoringDirection.kNone;
    }
}

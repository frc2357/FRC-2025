package frc.robot.controls.controllers;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;

public class ButtonboardController extends GenericHID implements Sendable {
    public enum ReefSide {
        None(0),
        A(1),
        B(2),
        C(3),
        D(4),
        E(5),
        F(6);

        public final int buttonValue;

        ReefSide(int buttonValue) {
            this.buttonValue = buttonValue;
        }
    }

    public enum ScoringLevel {
        None(0),
        L4(7),
        L3(8),
        L2(9),
        L1(10);

        public final int buttonValue;

        ScoringLevel(int buttonValue) {
            this.buttonValue = buttonValue;
        }
    }

    public enum ScoringDirection {
        None(0),
        Left(11),
        Right(12);

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
        return ReefSide.None;
    }

    public ScoringLevel getSelectedScoringLevel() {
        for (ScoringLevel lvl : ScoringLevel.values()) {
            if (lvl.buttonValue != 0 && getRawButton(lvl.buttonValue)) {
                return lvl;
            }
        }
        return ScoringLevel.None;
    }

    public ScoringDirection getSelectedScoringDirection() {
        for (ScoringDirection dir : ScoringDirection.values()) {
            if (dir.buttonValue != 0 && getRawButton(dir.buttonValue)) {
                return dir;
            }
        }
        return ScoringDirection.None;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("HID");
        builder.addStringProperty("Reef Side", () -> getSelectedReefSide().name(), null);
        builder.addStringProperty("Scoring Level", () -> getSelectedScoringLevel().name(), null);
        builder.addStringProperty("Scoring Direction", () -> getSelectedScoringDirection().name(), null);
    }
}

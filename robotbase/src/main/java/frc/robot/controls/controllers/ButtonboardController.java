package frc.robot.controls.controllers;

import static frc.robot.Constants.FIELD.REEF.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Robot;

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
    builder.addStringProperty(
      "Reef Side",
      () -> getSelectedReefSide().name(),
      null
    );
    builder.addStringProperty(
      "Scoring Level",
      () -> getSelectedScoringLevel().name(),
      null
    );
    builder.addStringProperty(
      "Scoring Direction",
      () -> getSelectedScoringDirection().name(),
      null
    );
  }

  public Pose2d getPoseFromGoal() {
    ReefSide goal = Robot.buttonboard.getSelectedReefSide();
    ScoringDirection scoringDirection =
      Robot.buttonboard.getSelectedScoringDirection();

    switch (scoringDirection) {
      case Left:
        switch (goal) {
          case A:
            return BRANCH_A;
          case B:
            return BRANCH_C;
          case C:
            return BRANCH_E;
          case D:
            return BRANCH_G;
          case E:
            return BRANCH_I;
          case F:
            return BRANCH_K;
          default:
            return new Pose2d(-1, -1, new Rotation2d(Units.Degrees.of(-1)));
        }
      case Right:
        switch (goal) {
          case A:
            return BRANCH_B;
          case B:
            return BRANCH_D;
          case C:
            return BRANCH_F;
          case D:
            return BRANCH_H;
          case E:
            return BRANCH_J;
          case F:
            return BRANCH_L;
          default:
            return new Pose2d(-1, -1, new Rotation2d(Units.Degrees.of(-1)));
        }
      default:
        return new Pose2d(-1, -1, new Rotation2d(Units.Degrees.of(-1)));
    }
  }
}

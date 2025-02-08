package frc.robot.controls.controllers;

import static frc.robot.Constants.FIELD.REEF.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;

public class ButtonboardController extends GenericHID implements Sendable {

  public enum ReefSide {
    None(-1),
    A(XboxController.Button.kA.value),
    B(XboxController.Button.kB.value),
    C(XboxController.Button.kX.value),
    D(XboxController.Button.kY.value),
    E(XboxController.Button.kBack.value),
    F(XboxController.Button.kStart.value);

    public final int buttonValue;

    ReefSide(int buttonValue) {
      this.buttonValue = buttonValue;
    }
  }

  public enum ScoringLevel {
    None(-1),
    L4(0),
    L3(90),
    L2(180),
    L1(270);

    public final int povValue;

    ScoringLevel(int povValue) {
      this.povValue = povValue;
    }
  }

  public enum ScoringDirection {
    None(-1),
    Left(XboxController.Button.kLeftBumper.value),
    Right(XboxController.Button.kRightBumper.value);

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
      if (side.buttonValue != -1 && getRawButton(side.buttonValue)) {
        return side;
      }
    }
    return ReefSide.None;
  }

  public ScoringLevel getSelectedScoringLevel() {
    for (ScoringLevel lvl : ScoringLevel.values()) {
      if (lvl.povValue != -1 && lvl.povValue == getPOV()) {
        return lvl;
      }
    }
    return ScoringLevel.None;
  }

  public ScoringDirection getSelectedScoringDirection() {
    for (ScoringDirection dir : ScoringDirection.values()) {
      if (dir.buttonValue != -1 && getRawButton(dir.buttonValue)) {
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

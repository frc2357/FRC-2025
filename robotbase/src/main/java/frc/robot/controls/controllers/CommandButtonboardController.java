package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandButtonboardController extends CommandGenericHID {
  // Button to indicate if roller mechanisms (triggers) are negative since triggers can only be positive
  // TODO: Determine if it would be better to make all axes use this for consistency
  private final int ROLLER_MECHANISM_NEGATIVE_INDICATOR_BUTTON = XboxController.Button.kLeftStick.value;

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

  public enum PanicControlAxis {
    None(-1),
    CoralIntake(XboxController.Axis.kRightTrigger.value),
    AlgaeIntake(XboxController.Axis.kLeftTrigger.value),
    Elevator(XboxController.Axis.kRightY.value),
    Laterator(XboxController.Axis.kRightX.value),
    AlgaePivot(XboxController.Axis.kLeftX.value),
    Climber(XboxController.Axis.kLeftY.value);

    public final int axisValue;

    PanicControlAxis(int axisValue) {
      this.axisValue = axisValue;
    }
  }

  public CommandButtonboardController(final int port) {
    super(port);
  }

  // Reef side button triggers
  public Trigger a() {
    return button(
      ReefSide.A.buttonValue,
      CommandScheduler.getInstance().getDefaultButtonLoop()
    );
  }

  public Trigger b() {
    return button(
      ReefSide.B.buttonValue,
      CommandScheduler.getInstance().getDefaultButtonLoop()
    );
  }

  public Trigger c() {
    return button(
      ReefSide.C.buttonValue,
      CommandScheduler.getInstance().getDefaultButtonLoop()
    );
  }

  public Trigger d() {
    return button(
      ReefSide.D.buttonValue,
      CommandScheduler.getInstance().getDefaultButtonLoop()
    );
  }

  public Trigger e() {
    return button(
      ReefSide.E.buttonValue,
      CommandScheduler.getInstance().getDefaultButtonLoop()
    );
  }

  public Trigger f() {
    return button(
      ReefSide.F.buttonValue,
      CommandScheduler.getInstance().getDefaultButtonLoop()
    );
  }

  public Trigger noSide() {
    return a().or(b()).or(c()).or(d()).or(e()).or(f()).negate();
  }

  // Scoring level dpad triggers
  public Trigger L1() {
    return pov(ScoringLevel.L1.povValue);
  }

  public Trigger L2() {
    return pov(ScoringLevel.L2.povValue);
  }

  public Trigger L3() {
    return pov(ScoringLevel.L3.povValue);
  }

  public Trigger L4() {
    return pov(ScoringLevel.L4.povValue);
  }

  public Trigger noLevel() {
    return L1().or(L2()).or(L3()).or(L4()).negate();
  }

  // Scoring direction button triggers
  public Trigger left() {
    return button(
      ScoringDirection.Left.buttonValue,
      CommandScheduler.getInstance().getDefaultButtonLoop()
    );
  }

  public Trigger right() {
    return button(
      ScoringDirection.Right.buttonValue,
      CommandScheduler.getInstance().getDefaultButtonLoop()
    );
  }

  public Trigger noDirection() {
    return left().or(right()).negate();
  }

  // Doesn't look like CommandGenericHID::axisGreaterThan takes negatives into account
  public Trigger coralIntakeAxis(double threshold) {
    return new Trigger(
      CommandScheduler.getInstance().getDefaultButtonLoop(),
      () -> Math.abs(getRawAxis(PanicControlAxis.CoralIntake.axisValue)) > threshold
    );
  }

  public double getCoralIntakeAxis() {
    return modifyRollerAxis(getRawAxis(PanicControlAxis.CoralIntake.axisValue));
  }

  public Trigger algaeIntakeAxis(double threshold) {
    return new Trigger(
      CommandScheduler.getInstance().getDefaultButtonLoop(),
      () -> Math.abs(getRawAxis(PanicControlAxis.AlgaeIntake.axisValue)) > threshold
    );
  }

  public double getAlgaeIntakeAxis() {
    return modifyRollerAxis(getRawAxis(PanicControlAxis.AlgaeIntake.axisValue));
  }

  public Trigger elevatorAxis(double threshold) {
    return new Trigger(
      CommandScheduler.getInstance().getDefaultButtonLoop(),
      () -> Math.abs(getRawAxis(PanicControlAxis.Elevator.axisValue)) > threshold
    );
  }

  public double getElevatorAxis() {
    return getRawAxis(PanicControlAxis.Elevator.axisValue);
  }

  public Trigger lateratorAxis(double threshold) {
    return new Trigger(
      CommandScheduler.getInstance().getDefaultButtonLoop(),
      () -> Math.abs(getRawAxis(PanicControlAxis.Laterator.axisValue)) > threshold
    );
  }

  public double getLateratorAxis() {
    return getRawAxis(PanicControlAxis.Laterator.axisValue);
  }

  public Trigger algaePivotAxis(double threshold) {
    return new Trigger(
      CommandScheduler.getInstance().getDefaultButtonLoop(),
      () -> Math.abs(getRawAxis(PanicControlAxis.AlgaePivot.axisValue)) > threshold
    );
  }

  public double getAlgaePivotAxis() {
    return getRawAxis(PanicControlAxis.AlgaePivot.axisValue);
  }

  public Trigger climberAxis(double threshold) {
    return new Trigger(
      CommandScheduler.getInstance().getDefaultButtonLoop(),
      () -> Math.abs(getRawAxis(PanicControlAxis.Climber.axisValue)) > threshold
    );
  }

  public double getClimberAxis() {
    return getRawAxis(PanicControlAxis.Climber.axisValue);
  }

  // Button to indicate if roller mechanisms (triggers) are negative since triggers can only be positive
  private double modifyRollerAxis(double value) {
    int modifier = getHID().getRawButton(ROLLER_MECHANISM_NEGATIVE_INDICATOR_BUTTON) ? -1 : 1;
    return modifier * value;
  }
}

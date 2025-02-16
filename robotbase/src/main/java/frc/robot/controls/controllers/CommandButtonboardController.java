package frc.robot.controls.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandButtonboardController extends CommandGenericHID {

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
}

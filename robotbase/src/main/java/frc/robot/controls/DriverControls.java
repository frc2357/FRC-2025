package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.descoring.AlgaeRemoverChooser;
import frc.robot.commands.descoring.RemoveAlgaeHigh;
import frc.robot.commands.descoring.RemoveAlgaeLow;
import frc.robot.commands.drive.DriveRobotRelative;
import frc.robot.commands.drive.FlipPerspective;
import frc.robot.commands.scoring.coral.CoralHome;
import frc.robot.commands.scoring.coral.CoralIntakeScoreConditional;
import frc.robot.commands.scoring.coral.CoralPreposeL2;
import frc.robot.commands.scoring.coral.CoralPreposeL3;
import frc.robot.commands.scoring.coral.CoralPreposeL4;

@SuppressWarnings("unused")
public class DriverControls {

  private CommandXboxController m_controller;

  private double m_deadband;

  private Trigger m_rightTrigger;
  private Trigger m_leftTrigger;

  public DriverControls(CommandXboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    m_rightTrigger = m_controller.axisGreaterThan(Axis.kRightTrigger.value, 0);
    m_leftTrigger = m_controller.axisGreaterThan(Axis.kLeftTrigger.value, 0);

    mapControls();
  }

  public double getRightStickYAxis() {
    return m_controller.getRightY();
  }

  public void mapControls() {
    m_controller
      .start()
      .onTrue(new InstantCommand(() -> Robot.swerve.seedFieldCentric()));

    // Manual Coral Scoring
    m_controller.rightBumper().onTrue(new CoralPreposeL3());
    m_leftTrigger.onTrue(new CoralHome());
    m_rightTrigger.onTrue(new CoralIntakeScoreConditional());
    m_controller.leftBumper().onTrue(new CoralPreposeL4());

    AlgaeRemoverChooser algaeRemoverChooser = new AlgaeRemoverChooser();
    //m_controller.a().onTrue(algaeRemoverChooser.getSelectCommand());
    m_controller
      .a()
      .toggleOnTrue(
        new RemoveAlgaeLow().finallyDo(() -> new CoralHome().schedule())
      );
    m_controller
      .b()
      .toggleOnTrue(
        new RemoveAlgaeHigh().finallyDo(() -> new CoralHome().schedule())
      );
    m_controller.x().onTrue(new CoralPreposeL2());
    m_controller.y().whileTrue(new DriveRobotRelative());

    m_controller.back().onTrue(new FlipPerspective());
  }

  public double getX() {
    double value = -modifyAxis(m_controller.getLeftX());
    return Math.copySign(Math.pow(value, 2), value);
  }

  public double getY() {
    double value = -modifyAxis(m_controller.getLeftY());
    return Math.copySign(Math.pow(value, 2), value);
  }

  public double getRotation() {
    return -modifyAxis(m_controller.getRightX());
  }

  public double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public double modifyAxis(double value) {
    return deadband(value, m_deadband);
  }
}

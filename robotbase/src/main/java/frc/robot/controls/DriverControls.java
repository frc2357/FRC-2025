package frc.robot.controls;

import static frc.robot.Constants.FIELD.REEF.BRANCH_F;
import static frc.robot.Constants.FIELD.REEF.BRANCH_I;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.descoring.RemoveAlgaeHigh;
import frc.robot.commands.descoring.RemoveAlgaeLow;
import frc.robot.commands.drive.DriveToPoseHandler.RouteAroundReef;
import frc.robot.commands.drive.DriveToReef;
import frc.robot.commands.intake.TeleopCoralIntake;
import frc.robot.commands.scoring.CoralHome;
import frc.robot.commands.scoring.CoralZero;
import frc.robot.commands.scoring.teleop.TeleopCoralScoreL2;
import frc.robot.commands.scoring.teleop.TeleopCoralScoreL3;
import frc.robot.commands.scoring.teleop.TeleopCoralScoreL4;
import frc.robot.controls.util.RumbleInterface;

public class DriverControls implements RumbleInterface {

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
    // Scoring
    m_controller
      .leftBumper()
      .onTrue(
        new TeleopCoralScoreL4(m_rightTrigger)
          .getCommand()
          .andThen(new CoralZero())
      );
    m_controller
      .rightBumper()
      .onTrue(
        new TeleopCoralScoreL3(m_rightTrigger)
          .getCommand()
          .andThen(new CoralZero())
      );
    m_controller
      .rightStick()
      .onTrue(
        new TeleopCoralScoreL2(m_rightTrigger)
          .getCommand()
          .andThen(new CoralZero())
      );

    // Intaking
    m_rightTrigger
      .and(() -> Robot.coralRunner.hasNoCoral())
      .onTrue(new TeleopCoralIntake(m_rightTrigger));

    // Remove algae
    m_controller.a().onTrue(new RemoveAlgaeLow(m_controller.a()));
    m_controller.y().onTrue(new RemoveAlgaeHigh(m_controller.b()));

    // Other
    m_leftTrigger.onTrue(new CoralHome().andThen(new CoralZero()));
    m_controller
      .back()
      .onTrue(
        new InstantCommand(() ->
          Robot.swerve.resetTranslation(
            Robot.camManager
              .getLastEstimatedPose()
              .getTranslation()
              .toTranslation2d()
          )
        )
      );
    m_controller
      .start()
      .onTrue(
        new InstantCommand(() -> Robot.swerve.resetHeading(Rotation2d.kZero))
      );
    m_controller
      .x()
      .whileTrue(new DriveToReef(RouteAroundReef.Fastest, BRANCH_I));
    m_controller
      .b()
      .whileTrue(new DriveToReef(RouteAroundReef.Fastest, BRANCH_F));
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

  @Override
  public void setRumble(double intensity) {
    m_controller.setRumble(RumbleType.kBothRumble, intensity);
  }
}

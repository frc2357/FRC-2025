package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.commands.drive.DriveToCoralStation;
import frc.robot.commands.drive.DriveToCoralStation.StationToGoTo;
import frc.robot.commands.drive.DriveToPoseHandler;
import frc.robot.commands.drive.DriveToPoseHandler.RouteAroundReef;
import frc.robot.commands.drive.VelDrive;
import frc.robot.commands.scoring.coral.CoralHumanPrepose;
import frc.robot.commands.scoring.coral.CoralScore;

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
    m_controller.a().whileTrue(new DriveToPoseHandler(RouteAroundReef.Fastest));
    m_controller
      .b()
      .whileTrue(
        new DriveToCoralStation(StationToGoTo.Fastest, RouteAroundReef.Fastest)
      );

    m_controller
      .x()
      .onTrue(
        new InstantCommand(() ->
          Robot.swerve.resetPose(
            REEF.BRANCH_C.plus(new Transform2d(0, -1, Rotation2d.kZero))
          )
        )
      );

    m_controller
      .start()
      .onTrue(new InstantCommand(() -> Robot.swerve.seedFieldCentric()));

    m_controller.b().whileTrue(new VelDrive());
    // Manual Coral Scoring
    // CoralHumanPrepose humanPrepose = new CoralHumanPrepose();
    // m_controller.leftBumper().onTrue(humanPrepose.getSelectCommand());
    // m_controller
    //   .leftTrigger()
    //   .onTrue(new CoralScore().andThen(humanPrepose.reset()));
  }

  public double getX() {
    return -modifyAxis(m_controller.getLeftX());
  }

  public double getY() {
    return -modifyAxis(m_controller.getLeftY());
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

package frc.robot.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.commands.drive.DriveToCoralStation;
import frc.robot.commands.drive.DriveToCoralStation.StationToGoTo;
import frc.robot.commands.drive.DriveToPoseHandler;
import frc.robot.commands.drive.DriveToPoseHandler.RouteAroundReef;
import frc.robot.commands.drive.FlipPerspective;
import frc.robot.commands.drive.VelDrive;
import frc.robot.commands.elevator.ElevatorHome;
import frc.robot.commands.elevator.ElevatorZero;
import frc.robot.commands.intake.AlgaeChooser;
import frc.robot.commands.intake.CoralIntake;
import frc.robot.commands.laterator.LateratorHome;
import frc.robot.commands.scoring.coral.CoralChooser;
import frc.robot.commands.scoring.coral.CoralHome;
import frc.robot.commands.scoring.coral.CoralScore;
import org.opencv.ml.Ml;

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
      .povUp()
      .negate()
      .and(m_controller.povRight().negate())
      .and(m_controller.x())
      .onTrue(new CoralHome());
    m_controller.povUp().and(m_controller.x()).onTrue(new ElevatorHome());
    m_controller.povRight().and(m_controller.x()).onTrue(new LateratorHome());
    m_controller
      .start()
      .onTrue(new InstantCommand(() -> Robot.swerve.seedFieldCentric()));

    m_leftTrigger.toggleOnTrue(new CoralIntake());
    // Manual Coral Scoring
    CoralChooser coralChooser = new CoralChooser();
    m_controller.rightBumper().onTrue(coralChooser.getLevelCommand());
    m_rightTrigger.onTrue(coralChooser.getSelectCommand());

    // AlgaeChooser algaeChooser = new AlgaeChooser();
    // m_controller.rightTrigger().onTrue(algaeChooser.getSelectCommand());

    m_controller.back().onTrue(new FlipPerspective());
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

package frc.robot.controls;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.commands.algaeKnocker.AlgaeKnockerSetSpeed;
import frc.robot.commands.algaeRunner.AlgaeRunnerSetSpeed;
import frc.robot.commands.descoring.AlgaeRemoverChooser;
import frc.robot.commands.descoring.RemoveAlgaeHigh;
import frc.robot.commands.descoring.RemoveAlgaeLow;
import frc.robot.commands.drive.DriveRobotRelative;
import frc.robot.commands.drive.DriveToCoralStation;
import frc.robot.commands.drive.DriveToCoralStation.StationToGoTo;
import frc.robot.commands.drive.DriveToPoseHandler;
import frc.robot.commands.drive.DriveToPoseHandler.RouteAroundReef;
import frc.robot.commands.drive.FlipPerspective;
import frc.robot.commands.drive.VelDrive;
import frc.robot.commands.elevator.ElevatorHome;
import frc.robot.commands.elevator.ElevatorSetDistance;
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
      .start()
      .onTrue(
        new InstantCommand(() ->
          Robot.swerve.resetPose(
            REEF.BRANCH_A.plus(new Transform2d(-0.2, 0, Rotation2d.kZero))
          )
        )
      );

    // Manual Coral Scoring
    CoralChooser coralChooser = new CoralChooser();
    m_controller.rightBumper().onTrue(coralChooser.getElevatorPreposeCommand());
    m_leftTrigger.onTrue(new CoralHome());
    m_rightTrigger.toggleOnTrue(coralChooser.getScoreCommand());
    m_controller.leftBumper().onTrue(coralChooser.selectL4());

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
    m_controller.x().onTrue(coralChooser.selectL3());
    m_controller
      .y()
      .whileTrue(
        new DriveToCoralStation(StationToGoTo.LeftSide, RouteAroundReef.Fastest)
      );

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

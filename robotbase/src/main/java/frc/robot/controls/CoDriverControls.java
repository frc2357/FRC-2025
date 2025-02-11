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
import frc.robot.commands.algaePivot.AlgaePivotAxis;
import frc.robot.commands.algaeRunner.AlgaeRunnerAxis;
import frc.robot.commands.drive.DriveToCoralStation;
import frc.robot.commands.drive.DriveToCoralStation.StationToGoTo;
import frc.robot.commands.drive.DriveToPoseHandler;
import frc.robot.commands.drive.DriveToPoseHandler.RouteAroundReef;

@SuppressWarnings("unused")
public class CoDriverControls {

  private CommandXboxController m_controller;

  private double m_deadband;

  private Trigger m_rightTrigger;
  private Trigger m_leftTrigger;

  public CoDriverControls(CommandXboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    m_rightTrigger = m_controller.axisGreaterThan(Axis.kRightTrigger.value, 0);
    m_leftTrigger = m_controller.axisGreaterThan(Axis.kLeftTrigger.value, 0);

    mapControls();
  }

  public void mapControls() {
    m_controller
      .axisMagnitudeGreaterThan(Axis.kLeftY.value, 0.01)
      .whileTrue(new AlgaePivotAxis(() -> modifyAxis(m_controller.getLeftY())));
    m_controller
      .axisMagnitudeGreaterThan(Axis.kRightY.value, 0.01)
      .whileTrue(
        new AlgaeRunnerAxis(() -> modifyAxis(m_controller.getRightY()))
      );
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

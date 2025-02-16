package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.coralRunner.CoralRunnerAxis;
import frc.robot.commands.elevator.ElevatorAxis;
import frc.robot.commands.laterator.LateratorAxis;

public class CodriverControls {

  private CommandXboxController m_controller;

  private double m_deadband;

  private Trigger m_rightTrigger;
  private Trigger m_leftTrigger;

  public CodriverControls(CommandXboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    m_rightTrigger = m_controller.axisGreaterThan(Axis.kRightTrigger.value, 0);
    m_leftTrigger = m_controller.axisGreaterThan(Axis.kLeftTrigger.value, 0);

    mapControls();
  }

  public void mapControls() {
    m_controller
      .povUp()
      .whileTrue(new ElevatorAxis(() -> modifyAxis(-m_controller.getRightY())));

    m_controller
      .povRight()
      .whileTrue(
        new LateratorAxis(() -> modifyAxis(-m_controller.getRightX()))
      );
    m_controller
      .povRight()
      .and(m_rightTrigger)
      .whileTrue(
        new CoralRunnerAxis(() -> modifyAxis(m_controller.getRightTriggerAxis())
        )
      );
    m_controller
      .povRight()
      .and(m_leftTrigger)
      .whileTrue(
        new CoralRunnerAxis(() -> modifyAxis(m_controller.getRightTriggerAxis())
        )
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

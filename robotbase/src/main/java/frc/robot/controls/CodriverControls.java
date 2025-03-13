package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LATERATOR;
import frc.robot.commands.algaeKnocker.AlgaeKnockerSetSpeed;
import frc.robot.commands.climber.ClimberAxis;
import frc.robot.commands.coralRunner.CoralRunnerAxis;
import frc.robot.commands.elevator.ElevatorAxis;
import frc.robot.commands.elevator.ElevatorHome;
import frc.robot.commands.laterator.LateratorAxis;
import frc.robot.commands.laterator.LateratorFullZero;
import frc.robot.commands.laterator.LateratorFullZero;
import frc.robot.commands.laterator.LateratorHome;
import frc.robot.commands.laterator.LateratorSetDistance;
import frc.robot.commands.scoring.coral.CoralHome;
import frc.robot.commands.scoring.coral.CoralPreposeL1;
import frc.robot.commands.scoring.coral.CoralPreposeL2;
import frc.robot.commands.scoring.coral.CoralPreposeL3;
import frc.robot.commands.scoring.coral.CoralPreposeL4;

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
    Trigger noDpad = m_controller
      .povUp()
      .negate()
      .and(m_controller.povRight().negate())
      .and(m_controller.povLeft().negate())
      .and(m_controller.povDown().negate());

    Trigger onlyLeft = m_controller
      .povUp()
      .negate()
      .and(m_controller.povRight().negate())
      .and(m_controller.povLeft())
      .and(m_controller.povDown().negate());

    Trigger onlyRight = m_controller
      .povUp()
      .negate()
      .and(m_controller.povRight())
      .and(m_controller.povLeft().negate())
      .and(m_controller.povDown().negate());

    Trigger onlyUp = m_controller
      .povUp()
      .and(m_controller.povRight().negate())
      .and(m_controller.povLeft().negate())
      .and(m_controller.povDown().negate());

    Trigger onlyDown = m_controller
      .povUp()
      .negate()
      .and(m_controller.povRight().negate())
      .and(m_controller.povLeft().negate())
      .and(m_controller.povDown());

    noDpad.and(m_controller.x()).onTrue(new CoralHome());
    m_controller.povUp().and(m_controller.x()).onTrue(new ElevatorHome());
    m_controller.povRight().and(m_controller.x()).onTrue(new LateratorHome());

    onlyUp.whileTrue(
      new ElevatorAxis(() -> modifyAxis(-m_controller.getRightY()))
    );

    onlyUp.and(m_controller.x().whileTrue(new ElevatorHome()));

    onlyLeft.and(m_controller.a()).whileTrue(new CoralPreposeL1());
    onlyLeft.and(m_controller.b()).whileTrue(new CoralPreposeL2());
    onlyLeft.and(m_controller.x()).whileTrue(new CoralPreposeL3());
    onlyLeft.and(m_controller.y()).whileTrue(new CoralPreposeL4());

    onlyRight.whileTrue(
      new LateratorAxis(() -> modifyAxis(-m_controller.getRightX()))
    );

    // onlyRight.onTrue(new LateratorZero());

    onlyRight
      .and(m_rightTrigger)
      .whileTrue(
        new CoralRunnerAxis(() -> -m_controller.getRightTriggerAxis())
      );
    onlyRight
      .and(m_leftTrigger)
      .whileTrue(new CoralRunnerAxis(() -> m_controller.getLeftTriggerAxis()));

    onlyRight.and(m_controller.a()).whileTrue(new AlgaeKnockerSetSpeed(0.25));
    onlyRight.and(m_controller.b()).whileTrue(new AlgaeKnockerSetSpeed(-0.25));
    onlyDown.whileTrue(new ClimberAxis(() -> -m_controller.getRightX()));
    onlyDown.onTrue(new LateratorSetDistance(LATERATOR.SETPOINTS.L4_PREPOSE));
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

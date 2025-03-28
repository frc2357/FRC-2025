package frc.robot.controls;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.algaeKnocker.AlgaeKnockerSetSpeed;
import frc.robot.commands.climberPivot.ClimberPivotAxis;
import frc.robot.commands.climberPivot.ClimberPivotSetSpeed;
import frc.robot.commands.climberWinch.ClimberWinchAxis;
import frc.robot.commands.climberWinch.ClimberWinchSetSpeed;
import frc.robot.commands.coralRunner.CoralRunnerAxis;
import frc.robot.commands.elevator.ElevatorAmpLimitZero;
import frc.robot.commands.elevator.ElevatorAxis;
import frc.robot.commands.elevator.ElevatorHome;
import frc.robot.commands.laterator.LateratorAxis;
import frc.robot.commands.laterator.LateratorFullZero;
import frc.robot.commands.laterator.LateratorHome;
import frc.robot.commands.laterator.LateratorSetDistance;
import frc.robot.commands.laterator.LateratorZero;
import frc.robot.commands.scoring.CoralHome;
import frc.robot.commands.scoring.CoralZero;
import frc.robot.controls.util.RumbleInterface;

public class CodriverControls implements RumbleInterface {

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
    Trigger noLetterButtons = m_controller
      .a()
      .negate()
      .and(m_controller.b().negate())
      .and(m_controller.x().negate())
      .and(m_controller.y().negate());

    m_controller
      .start()
      .onTrue(
        new InstantCommand(() -> CommandScheduler.getInstance().cancelAll())
      );

    noDpad
      .and(m_controller.x())
      .whileTrue(new CoralHome().andThen(new CoralZero()));
    noDpad.and(m_controller.a()).whileTrue(new CoralZero());
    noDpad
      .and(m_controller.leftBumper())
      .whileTrue(
        new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.INTAKE_PREPOSE)
      );
    noDpad
      .and(m_controller.rightBumper())
      .whileTrue(
        new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.L3_PREPOSE)
      );

    onlyUp.whileTrue(
      new ElevatorAxis(() -> modifyAxis(-m_controller.getRightY()))
    );
    onlyUp
      .and(m_controller.x())
      .whileTrue(new ElevatorHome().andThen(new ElevatorAmpLimitZero()));
    onlyUp.and(m_controller.a()).whileTrue(new ElevatorAmpLimitZero());

    onlyRight
      .and(noLetterButtons)
      .whileTrue(
        new LateratorAxis(() -> modifyAxis(-m_controller.getRightX()))
      );

    onlyRight
      .and(m_rightTrigger)
      .whileTrue(
        new CoralRunnerAxis(() -> -m_controller.getRightTriggerAxis())
      );
    onlyRight
      .and(m_leftTrigger)
      .whileTrue(new CoralRunnerAxis(() -> m_controller.getLeftTriggerAxis()));

    onlyRight.and(m_controller.y()).whileTrue(new LateratorFullZero());
    onlyRight
      .and(m_controller.x())
      .whileTrue(new LateratorHome().andThen(new LateratorZero()));
    onlyRight.and(m_controller.a()).whileTrue(new LateratorZero());
    onlyRight
      .and(m_controller.leftBumper())
      .whileTrue(new AlgaeKnockerSetSpeed(0.25));
    onlyRight
      .and(m_controller.rightBumper())
      .whileTrue(new AlgaeKnockerSetSpeed(-0.25));

    onlyLeft.whileTrue(
      new ClimberPivotAxis(
        () ->
          m_controller.getLeftTriggerAxis() - m_controller.getRightTriggerAxis()
      )
    );
    onlyLeft.whileTrue(
      new ClimberWinchAxis(
        () -> m_controller.getRightX(),
        () -> m_controller.getRightY()
      )
    );
    onlyLeft.and(m_controller.a()).whileTrue(new ClimberWinchSetSpeed(0.8));
    onlyLeft.and(m_controller.y()).whileTrue(new ClimberWinchSetSpeed(-0.8));
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

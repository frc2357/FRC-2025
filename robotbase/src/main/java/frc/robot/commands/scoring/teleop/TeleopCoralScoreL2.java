package frc.robot.commands.scoring.teleop;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;
import frc.robot.commands.util.PressToContinue;

public class TeleopCoralScoreL2 {

  private boolean m_scoreQueued;
  private Trigger m_scoreTrigger;

  public TeleopCoralScoreL2(Trigger scoreTrigger) {
    m_scoreTrigger = scoreTrigger;
  }

  public Command getCommand() {
    return new ParallelCommandGroup(
      new SequentialCommandGroup(
        new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.HOME).onlyIf(
          () ->
            Robot.laterator
              .getDistance()
              .lt(
                Constants.LATERATOR.SETPOINTS.HOME.minus(Units.Inches.of(0.5))
              )
        ),
        new ElevatorSetDistance(
          Constants.ELEVATOR.SETPOINTS.L2_PREPOSE.plus(
            Units.Inches.of(
              SmartDashboard.getNumber("Elevator Setpoint Modifier", 0)
            )
          )
        ),
        new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.L2_PREPOSE),
        new WaitUntilCommand(() -> m_scoreQueued),
        new TeleopCoralConfirmScore(
          Constants.CORAL_RUNNER.SCORING_PERCENT_OTHER
        )
      ),
      new SequentialCommandGroup(
        new InstantCommand(() -> m_scoreQueued = false),
        new PressToContinue(m_scoreTrigger),
        new InstantCommand(() -> m_scoreQueued = true)
      )
    );
  }
}

package frc.robot.commands.scoring.teleop;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;
import frc.robot.commands.util.PressToContinue;

public class TeleopCoralScoreL4 extends SequentialCommandGroup {

  public TeleopCoralScoreL4(Trigger scoreTrigger) {
    super(
      new ElevatorSetDistance(
        Constants.ELEVATOR.SETPOINTS.L4_PREPOSE.plus(
          Units.Inches.of(
            SmartDashboard.getNumber("Elevator Setpoint Modifier", 0)
          )
        )
      ),
      new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.L2_PREPOSE),
      new PressToContinue(scoreTrigger),
      new TeleopCoralConfirmScore(Constants.CORAL_RUNNER.SCORING_PERCENT_L4)
    );
  }
}

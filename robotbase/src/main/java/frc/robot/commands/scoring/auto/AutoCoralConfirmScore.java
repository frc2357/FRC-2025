package frc.robot.commands.scoring.auto;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;
import frc.robot.commands.scoring.CoralHome;

public class AutoCoralConfirmScore extends SequentialCommandGroup {

  public AutoCoralConfirmScore(Dimensionless coralRunnerSpeed) {
    super(
      new WaitCommand(
        Constants.CORAL_RUNNER.AUTO_SCORING_WAIT_TIME
      ).withDeadline(
        new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.SCORING_PERCENT_L4)
      ),
      new CoralHome()
    );
  }
}

package frc.robot.commands.scoring.teleop;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;
import frc.robot.commands.scoring.CoralHome;

public class TeleopCoralConfirmScore extends SequentialCommandGroup {

  public TeleopCoralConfirmScore(Dimensionless coralRunnerSpeed) {
    super(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> Robot.coralRunner.hasNoCoral()),
          new WaitCommand(Constants.CORAL_RUNNER.TELEOP_SCORING_WAIT_TIME)
        ),
        new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.SCORING_PERCENT_L4)
      ),
      new CoralHome(() -> false)
    );
  }
}

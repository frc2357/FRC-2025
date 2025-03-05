package frc.robot.commands.scoring.coral;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;
import frc.robot.commands.coralRunner.CoralRunnerStop;
import frc.robot.commands.laterator.LateratorSetDistance;
import java.util.function.Supplier;

public class CoralScore extends SequentialCommandGroup {

  public CoralScore(Supplier<Distance> lateratorDistance) {
    super(
      new LateratorSetDistance(lateratorDistance),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitUntilCommand(Robot.coralRunner::isOuttakeBeamBroken),
          new WaitCommand(Constants.CORAL_RUNNER.SCORING_WAIT_TIME)
        ),
        new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.SCORING_PERCENT)
      ),
      new CoralRunnerStop(),
      new CoralHome()
    );
  }
}

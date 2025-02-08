package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetVelocity;
import frc.robot.commands.coralRunner.CoralRunnerStop;

public class CoralScore extends SequentialCommandGroup {

  public CoralScore() {
    super(
      new CoralRunnerSetVelocity(
        Constants.CORAL_RUNNER.SCORING_VELOCITY,
        false
      ).until(Robot.coralRunner::isOuttakeBeamBroken),
      new WaitCommand(Constants.CORAL_RUNNER.SCORING_WAIT_TIME),
      new CoralRunnerStop(),
      new CoralHome()
    );
  }
}

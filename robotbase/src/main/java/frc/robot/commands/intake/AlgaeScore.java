package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ALGAE_PIVOT;
import frc.robot.Constants.ALGAE_RUNNER;
import frc.robot.commands.algaeRunner.AlgaeRunnerSetSpeed;

public class AlgaeScore extends SequentialCommandGroup {

  public AlgaeScore() {
    super(
      new AlgaeRunnerSetSpeed(ALGAE_RUNNER.ALGAE_EJECTOR_SPEED)
        .deadlineFor(new WaitCommand(ALGAE_PIVOT.ALGAE_SCORE_TIME))
        .andThen(new AlgaeRetract())
    );
  }
}

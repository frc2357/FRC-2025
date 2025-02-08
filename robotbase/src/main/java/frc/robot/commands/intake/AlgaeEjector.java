package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ALGAE_RUNNER;
import frc.robot.commands.algaeRunner.AlgaeRunnerSetSpeed;

public class AlgaeEjector extends ParallelCommandGroup {

  public AlgaeEjector() {
    super(new AlgaeRunnerSetSpeed(ALGAE_RUNNER.ALGAE_EJECTOR_SPEED));
  }
}

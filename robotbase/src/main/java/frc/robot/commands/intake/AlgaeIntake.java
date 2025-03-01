package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ALGAE_RUNNER;
import frc.robot.commands.algaeRunner.AlgaeRunnerSetSpeed;

public class AlgaeIntake extends ParallelCommandGroup {

  public AlgaeIntake() {
    super(
      new AlgaeRunnerSetSpeed(ALGAE_RUNNER.ALGAE_INTAKE_SPEED).alongWith(
        new AlgaeDeploy()
      )
    );
  }
}

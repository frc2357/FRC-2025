package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;

public class CoralIntake extends ParallelCommandGroup {

  public CoralIntake() {
    super(
      new CoralPreposeIntake(),
      new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.FAST_INTAKE_PERCENT).until(
        Robot.coralRunner::isIntakeBeamBroken
      )
    );
  }
}

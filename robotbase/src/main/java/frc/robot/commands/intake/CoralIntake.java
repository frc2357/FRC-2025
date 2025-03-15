package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.CORAL_RUNNER;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;

public class CoralIntake extends ParallelCommandGroup {

  public CoralIntake() {
    super(
      new CoralPreposeIntake(),
      new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.FAST_INTAKE_PERCENT)
        .until(Robot.coralRunner::isIntakeBeamBroken)
        .until(() -> Robot.coralRunner.isStalling())
        .andThen(
          new CoralRunnerSetSpeed(CORAL_RUNNER.BACK_OUT_PERCENT).withDeadline(
            new WaitCommand(0.4)
          )
        )
    );
  }
}

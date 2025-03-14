package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.CORAL_RUNNER;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;
import frc.robot.commands.scoring.CoralHome;

public class CoralIntake extends Command {

  public CoralIntake() {}

  @Override
  public void initialize() {
    new CoralPreposeIntake()
      .alongWith(
        new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.FAST_INTAKE_PERCENT)
          .until(Robot.coralRunner::isIntakeBeamBroken)
          .until(() -> Robot.coralRunner.isStalling())
          .andThen(
            new CoralRunnerSetSpeed(CORAL_RUNNER.BACK_OUT_PERCENT).withDeadline(
              new WaitCommand(0.4)
            )
          )
      )
      .finallyDo(() ->
        new CoralHome()
          .alongWith(
            new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.SLOW_INTAKE_PERCENT)
              .until(Robot.coralRunner::isOuttakeBeamBroken)
              .withDeadline(new WaitCommand(1))
          )
          .schedule()
      )
      .schedule();
  }

  @Override
  public boolean isFinished() {
    return Robot.coralRunner.isIntakeBeamBroken();
  }

  @Override
  public void end(boolean interrupted) {}
}

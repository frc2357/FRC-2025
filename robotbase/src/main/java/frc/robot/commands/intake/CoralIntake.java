package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;

public class CoralIntake extends Command {

  public CoralIntake() {}

  @Override
  public void initialize() {
    new CoralPreposeIntake()
      .alongWith(
        new CoralRunnerSetSpeed(
          Constants.CORAL_RUNNER.FAST_INTAKE_PERCENT
        ).until(Robot.coralRunner::isIntakeBeamBroken)
      )
      .schedule();
  }

  @Override
  public boolean isFinished() {
    return Robot.coralRunner.isIntakeBeamBroken();
  }

  @Override
  public void end(boolean interrupted) {
    new CoralIntakeRetract().schedule();
  }
}

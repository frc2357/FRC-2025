package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;
import frc.robot.commands.scoring.coral.CoralHome;

public class CoralIntakeRetract extends ParallelCommandGroup {

  public CoralIntakeRetract() {
    super(
      new CoralHome(),
      new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.SLOW_INTAKE_PERCENT)
        .until(Robot.coralRunner::isOuttakeBeamBroken)
        .withDeadline(new WaitCommand(1))
    );
  }
}

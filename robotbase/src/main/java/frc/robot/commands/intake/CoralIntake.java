package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetVelocity;
import frc.robot.commands.scoring.CoralHome;

public class CoralIntake extends SequentialCommandGroup {

  public CoralIntake() {
    super(
      new CoralRunnerSetVelocity(
        Constants.CORAL_RUNNER.FAST_INTAKE_VELOCITY
      ).until(Robot.coralRunner::isIntakeBeamBroken),
      new ParallelCommandGroup(
        new CoralHome(),
        new CoralRunnerSetVelocity(
          Constants.CORAL_RUNNER.SLOW_INTAKE_VELOCITY
        ).until(Robot.coralRunner::isOuttakeBeamBroken)
      )
    );
  }
}

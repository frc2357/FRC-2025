package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;
import frc.robot.commands.scoring.coral.CoralHome;

public class CoralIntake extends SequentialCommandGroup {

  public CoralIntake() {
    super(
      new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.FAST_INTAKE_PERCENT).until(
        Robot.coralRunner::isIntakeBeamBroken
      ),
      new ParallelCommandGroup(
        new CoralHome(),
        new CoralRunnerSetSpeed(
          Constants.CORAL_RUNNER.SLOW_INTAKE_PERCENT
        ).until(Robot.coralRunner::isOuttakeBeamBroken)
      )
    );
  }
}

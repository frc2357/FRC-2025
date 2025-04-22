package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.coralRunner.CoralRunnerSetSpeed;
import frc.robot.commands.util.PressToContinue;

public class TeleopCoralIntake extends SequentialCommandGroup {

  public TeleopCoralIntake(Trigger complete) {
    super(
      new ParallelDeadlineGroup(
        new ParallelRaceGroup(
          new PressToContinue(complete),
          new WaitUntilCommand(
            () ->
              Robot.coralRunner.isIntakeBeamBroken() ||
              Robot.coralRunner.isStalling()
          )
        ),
        new CoralPreposeIntake(),
        new CoralRunnerSetSpeed(Constants.CORAL_RUNNER.FAST_INTAKE_PERCENT)
      ),
      new CoralRetract()
    );
  }
}

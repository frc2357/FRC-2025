package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ALGAE_PIVOT;
import frc.robot.Robot;
import frc.robot.commands.algaePivot.AlgaePivotSetSpeed;

public class AlgaeRetract extends SequentialCommandGroup {

  public AlgaeRetract() {
    super(
      new AlgaePivotSetSpeed(ALGAE_PIVOT.RETRACT_SPEED)
        .deadlineFor(
          new WaitCommand(ALGAE_PIVOT.ALGAE_MOVEMENT_MIN_TIME).andThen(
            new WaitUntilCommand(() -> Robot.algaePivot.isStalling())
          )
        )
        .andThen(
          new AlgaePivotSetSpeed(-ALGAE_PIVOT.ALGAE_BACKOFF_SPEED).deadlineFor(
            new WaitCommand(ALGAE_PIVOT.ALGAE_BACKOFF_TIME)
          )
        )
    );
  }
}

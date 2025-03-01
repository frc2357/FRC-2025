package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ALGAE_PIVOT;
import frc.robot.Robot;

public class AlgaeRetract extends SequentialCommandGroup {

  public AlgaeRetract() {
    super(
      new AlgaePivotSetSpeed(ALGAE_PIVOT.RETRACT_SPEED).until(() ->
        Robot.algaePivot.isStalling()
      )
    );
  }
}

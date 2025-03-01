package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ALGAE_PIVOT;
import frc.robot.Robot;

public class AlgaeDeploy extends SequentialCommandGroup {

  public AlgaeDeploy() {
    super(
      new AlgaePivotSetSpeed(ALGAE_PIVOT.DEPLOY_SPEED).until(() ->
        Robot.algaePivot.isStalling()
      )
    );
  }
}

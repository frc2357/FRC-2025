package frc.robot.commands.laterator;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class LateratorToSafeSpot extends ConditionalCommand {

  public LateratorToSafeSpot() {
    super(
      new LateratorSetDistance(
        Constants.LATERATOR.SETPOINTS.MAX_SAFE_SCORING_EXTENSION
      ),
      new InstantCommand(),
      () ->
        Robot.laterator
          .getDistance()
          .lt(Constants.LATERATOR.SETPOINTS.MAX_SAFE_SCORING_EXTENSION)
    );
  }
}

package frc.robot.commands.laterator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.function.Supplier;

public class LateratorSetDistance extends Command {

  private Supplier<Distance> m_distance;

  public LateratorSetDistance(Supplier<Distance> distance) {
    m_distance = distance;
    addRequirements(Robot.laterator);
  }

  @Override
  public void execute() {
    if (
      Robot.swerve
        .getTranslationalVelocity()
        .lte(Constants.SWERVE.ROBOT_NO_TIP_SPEED) ||
      Robot.laterator.isGoingDown()
    ) {
      Robot.laterator.setTargetDistance(m_distance.get());
    } else {
      Robot.laterator.setTargetDistance(Robot.laterator.getDistance());
    }
  }

  @Override
  public boolean isFinished() {
    return Robot.laterator.isAtTarget();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.laterator.stop();
  }
}

package frc.robot.commands.laterator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LateratorSetDistance extends Command {

  private Distance m_distance;

  public LateratorSetDistance(Distance distance) {
    m_distance = distance;
    addRequirements(Robot.laterator);
  }

  @Override
  public void initialize() {
    Robot.laterator.setTargetDistance(m_distance);
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

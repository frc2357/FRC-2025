package frc.robot.commands.laterator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LateratorSetSpeed extends Command {

  double m_percent;

  public LateratorSetSpeed(double percent) {
    m_percent = percent;
    addRequirements(Robot.laterator);
  }

  @Override
  public void initialize() {
    Robot.laterator.setSpeed(m_percent);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.laterator.stop();
  }
}

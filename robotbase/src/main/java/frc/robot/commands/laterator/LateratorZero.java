package frc.robot.commands.laterator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LATERATOR;
import frc.robot.Robot;

public class LateratorZero extends Command {

  Timer m_timer;

  public LateratorZero() {
    addRequirements(Robot.laterator);
    m_timer = new Timer();
  }

  @Override
  public void initialize() {
    m_timer.start();
    Robot.laterator.setSpeed(LATERATOR.ZERO_SPEED);
  }

  @Override
  public boolean isFinished() {
    return (
      (m_timer.hasElapsed(LATERATOR.STALL_WAIT_TIME) &&
        Robot.laterator.ReachedStallLimit()) ||
      Robot.laterator.isAtZero()
    );
  }

  @Override
  public void end(boolean interrupted) {
    Robot.laterator.stop();
    if (!interrupted && Robot.laterator.isAtZero()) {
      Robot.laterator.setZero();
    }
    m_timer.stop();
    m_timer.reset();
  }
}

package frc.robot.commands.coralRunner;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class CoralRunnerSetVelocity extends Command {

  private AngularVelocity m_velocity;
  private boolean m_stopOnEnd;

  public CoralRunnerSetVelocity(AngularVelocity velocity, boolean stopOnEnd) {
    m_stopOnEnd = stopOnEnd;
    m_velocity = velocity;
    addRequirements(Robot.coralRunner);
  }

  public CoralRunnerSetVelocity(AngularVelocity velocity) {
    m_stopOnEnd = true;
    m_velocity = velocity;
    addRequirements(Robot.coralRunner);
  }

  @Override
  public void initialize() {
    Robot.coralRunner.setTargetVelocity(m_velocity);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (m_stopOnEnd) {
      Robot.coralRunner.stop();
    }
  }
}

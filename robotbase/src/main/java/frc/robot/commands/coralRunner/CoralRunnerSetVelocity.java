package frc.robot.commands.coralRunner;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class CoralRunnerSetVelocity extends Command {

  private AngularVelocity m_velocity;

  public CoralRunnerSetVelocity(AngularVelocity velocity) {
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
    Robot.coralRunner.stop();
  }
}

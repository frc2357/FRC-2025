package frc.robot.commands.coralRunner;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class CoralRunnerSetSpeed extends Command {

  public static int running = 0;
  private Dimensionless m_percent;

  public CoralRunnerSetSpeed(Dimensionless percent) {
    m_percent = percent;
    addRequirements(Robot.coralRunner);
  }

  @Override
  public void initialize() {
    running++;
    Robot.coralRunner.setSpeed(m_percent);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    running--;
    Robot.coralRunner.stop();
  }
}

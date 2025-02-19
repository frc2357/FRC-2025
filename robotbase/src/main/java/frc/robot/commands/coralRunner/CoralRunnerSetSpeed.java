package frc.robot.commands.coralRunner;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class CoralRunnerSetSpeed extends Command {

  private Dimensionless m_percent;

  public CoralRunnerSetSpeed(Dimensionless percent) {
    m_percent = percent;
    addRequirements(Robot.coralRunner);
  }

  @Override
  public void initialize() {
    Robot.coralRunner.setSpeed(m_percent);
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

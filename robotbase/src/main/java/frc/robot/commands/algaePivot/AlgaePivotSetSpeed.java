package frc.robot.commands.algaePivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AlgaePivotSetSpeed extends Command {

  private double m_speed;

  public AlgaePivotSetSpeed(double speed) {
    m_speed = speed;
    addRequirements(Robot.algaePivot);
  }

  @Override
  public void initialize() {
    Robot.algaePivot.setSpeed(m_speed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.algaePivot.stop();
  }
}

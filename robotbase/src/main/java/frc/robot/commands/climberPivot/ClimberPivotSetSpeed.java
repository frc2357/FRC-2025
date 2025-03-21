package frc.robot.commands.climberPivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberPivotSetSpeed extends Command {

  private double m_speed;

  public ClimberPivotSetSpeed(double speed) {
    addRequirements(Robot.climberPivot);
    m_speed = speed;
  }

  @Override
  public void initialize() {
    Robot.climberPivot.setSpeed(m_speed);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

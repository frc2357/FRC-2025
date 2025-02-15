package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberSetSpeed extends Command {

  private double m_speed;

  public ClimberSetSpeed(double speed) {
    addRequirements(Robot.climber);
    m_speed = speed;
  }

  @Override
  public void initialize() {
    Robot.climber.setSpeed(m_speed);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

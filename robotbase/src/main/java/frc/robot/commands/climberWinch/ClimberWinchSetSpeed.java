package frc.robot.commands.climberWinch;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberWinchSetSpeed extends Command {

  private double m_speed;

  public ClimberWinchSetSpeed(double speed) {
    addRequirements(Robot.climberWinch);
    m_speed = speed;
  }

  @Override
  public void initialize() {
    Robot.climberWinch.setSpeed(m_speed);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}

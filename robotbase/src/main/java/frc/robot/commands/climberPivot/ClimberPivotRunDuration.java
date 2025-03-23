package frc.robot.commands.climberPivot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberPivotRunDuration extends Command {

  private Time m_duration;
  private double m_speed;
  private Timer m_timer;

  public void climberPivotRunDuration(Time duration, double speed) {
    addRequirements(Robot.climberPivot);
    m_duration = duration;
    m_speed = speed;
    m_timer = new Timer();
  }

  @Override
  public void initialize() {
    Robot.climberPivot.setSpeed(m_speed);
    m_timer.restart();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration.in(Seconds));
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climberPivot.stop();
  }
}

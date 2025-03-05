package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Robot;

public class ElevatorZero extends Command {

  private Timer m_timer;

  public ElevatorZero() {
    addRequirements(Robot.elevator);
    m_timer = new Timer();
  }

  @Override
  public void initialize() {
    Robot.elevator.setSpeed(Constants.ELEVATOR.ZERO_SPEED);
    m_timer.start();
  }

  @Override
  public boolean isFinished() {
    return (
      m_timer.hasElapsed(ELEVATOR.ZERO_TIME.in(Seconds)) &&
      Robot.elevator.isAtZero()
    );
  }

  @Override
  public void end(boolean interrupted) {
    Robot.elevator.stop();
    if (!interrupted) {
      Robot.elevator.setZero();
    }
  }
}

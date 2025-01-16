package frc.robot.commands.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorSetDistance extends Command {

  private Distance m_distance;

  public ElevatorSetDistance(Distance distance) {
    m_distance = distance;
    addRequirements(Robot.elevator);
  }

  @Override
  public void initialize() {
    Robot.elevator.setTargetDistance(m_distance);
  }

  @Override
  public boolean isFinished() {
    return Robot.elevator.isAtSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.elevator.stop();
  }
}

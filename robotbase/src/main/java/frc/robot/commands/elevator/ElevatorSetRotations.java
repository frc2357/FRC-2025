package frc.robot.commands.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorSetRotations extends Command {

  private Angle m_rotations;

  public ElevatorSetRotations(Angle rotations) {
    m_rotations = rotations;
    addRequirements(Robot.elevator);
  }

  @Override
  public void initialize() {
    Robot.elevator.setTargetRotations(m_rotations);
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

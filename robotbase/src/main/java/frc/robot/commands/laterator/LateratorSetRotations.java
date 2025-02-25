package frc.robot.commands.laterator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LateratorSetRotations extends Command {

  private Angle m_rotations;

  public LateratorSetRotations(Angle rotations) {
    m_rotations = rotations;
    addRequirements(Robot.laterator);
  }

  @Override
  public void initialize() {
    Robot.laterator.setTargetRotations(m_rotations);
  }

  @Override
  public boolean isFinished() {
    return Robot.laterator.isAtTarget();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.laterator.stop();
  }
}

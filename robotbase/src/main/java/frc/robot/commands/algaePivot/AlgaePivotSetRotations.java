package frc.robot.commands.algaePivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AlgaePivotSetRotations extends Command {

  private Angle m_rotations;

  public AlgaePivotSetRotations(Angle rotations) {
    m_rotations = rotations;
    addRequirements(Robot.algaePivot);
  }

  @Override
  public void initialize() {
    Robot.algaePivot.setTargetRotations(m_rotations);
  }

  @Override
  public boolean isFinished() {
    return Robot.algaePivot.isAtTarget();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.algaePivot.stop();
  }
}

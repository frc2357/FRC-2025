package frc.robot.commands.algaePivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AlgaePivotSetAngle extends Command {

  private Angle m_angle;

  public AlgaePivotSetAngle(Angle angle) {
    m_angle = angle;
    addRequirements(Robot.algaePivot);
  }

  @Override
  public void initialize() {
    Robot.algaePivot.setTargetAngle(m_angle);
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

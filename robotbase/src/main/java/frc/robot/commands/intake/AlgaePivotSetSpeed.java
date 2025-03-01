package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AlgaePivotSetSpeed extends Command {

  private Dimensionless m_percent;

  public AlgaePivotSetSpeed(Dimensionless percent) {
    m_percent = percent;
    addRequirements(Robot.algaePivot);
  }

  @Override
  public void initialize() {
    Robot.algaePivot.setSpeed(m_percent.in(Percent));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.algaePivot.stop();
  }
}

package frc.robot.commands.algaePivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class AlgaePivotAxis extends Command {

  private AxisInterface m_axis;

  public AlgaePivotAxis(AxisInterface axis) {
    m_axis = axis;
    addRequirements(Robot.algaeRunner);
  }

  @Override
  public void execute() {
    double axisValue = m_axis.getValue();
    Robot.algaePivot.setAxisSpeed(axisValue);
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

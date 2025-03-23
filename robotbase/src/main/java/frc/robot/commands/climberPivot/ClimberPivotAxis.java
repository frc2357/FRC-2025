package frc.robot.commands.climberPivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class ClimberPivotAxis extends Command {

  private AxisInterface m_axis;

  public ClimberPivotAxis(AxisInterface axis) {
    m_axis = axis;
    addRequirements(Robot.climberPivot);
  }

  @Override
  public void execute() {
    double axisValue = m_axis.getValue();
    Robot.climberPivot.setAxisSpeed(Math.pow(axisValue, 3));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climberPivot.stop();
  }
}

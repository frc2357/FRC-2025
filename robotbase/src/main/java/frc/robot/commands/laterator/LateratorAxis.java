package frc.robot.commands.laterator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class LateratorAxis extends Command {

  private AxisInterface m_axis;

  public LateratorAxis(AxisInterface axis) {
    m_axis = axis;
    addRequirements(Robot.laterator);
  }

  @Override
  public void execute() {
    double axisValue = m_axis.getValue();
    Robot.laterator.setAxisSpeed(axisValue);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.laterator.stop();
  }
}

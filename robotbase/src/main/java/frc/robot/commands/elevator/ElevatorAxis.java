package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class ElevatorAxis extends Command {

  private AxisInterface m_axis;

  public ElevatorAxis(AxisInterface axis) {
    m_axis = axis;
    addRequirements(Robot.elevator);
  }

  @Override
  public void execute() {
    double axisValue = m_axis.getValue();
    Robot.elevator.setAxisSpeed(axisValue);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}

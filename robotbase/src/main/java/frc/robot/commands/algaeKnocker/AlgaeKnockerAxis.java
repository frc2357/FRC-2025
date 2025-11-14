package frc.robot.commands.algaeKnocker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class AlgaeKnockerAxis extends Command {

  private AxisInterface m_axis;

  public AlgaeKnockerAxis(AxisInterface axis) {
    m_axis = axis;
    addRequirements(Robot.algaeKnocker);
  }

  @Override
  public void execute() {
    double axisValue = m_axis.getValue();
    Robot.algaeKnocker.setAxisSpeed(axisValue);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.algaeKnocker.stop();
  }
}

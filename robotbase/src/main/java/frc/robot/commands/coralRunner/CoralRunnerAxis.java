package frc.robot.commands.coralRunner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class CoralRunnerAxis extends Command {

  private AxisInterface m_axis;

  public CoralRunnerAxis(AxisInterface axis) {
    m_axis = axis;
    addRequirements(Robot.coralRunner);
  }

  @Override
  public void execute() {
    double axisValue = m_axis.getValue();
    Robot.coralRunner.setAxisSpeed(axisValue);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.coralRunner.stop();
  }
}

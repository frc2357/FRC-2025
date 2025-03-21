package frc.robot.commands.climberWinch;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class ClimberWinchAxis extends Command {

  private AxisInterface m_axis;

  public ClimberWinchAxis(AxisInterface axis) {
    m_axis = axis;
    addRequirements(Robot.climberWinch);
  }

  @Override
  public void execute() {
    double axisValue = m_axis.getValue();
    Robot.climberWinch.setAxisSpeed(Math.pow(axisValue, 3));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.climberWinch.stop();
  }
}

package frc.robot.commands.climberWinch;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.controls.util.AxisInterface;

public class ClimberWinchAxis extends Command {

  private AxisInterface m_xAxis;
  private AxisInterface m_yAxis;

  public ClimberWinchAxis(AxisInterface xAxis, AxisInterface yAxis) {
    m_xAxis = xAxis;
    m_yAxis = yAxis;
    addRequirements(Robot.climberWinch);
  }

  @Override
  public void execute() {
    double xAxisSpeed = m_xAxis.getValue();
    double yAxisSpeed = m_yAxis.getValue();

    double leftSpeed = (-xAxisSpeed + yAxisSpeed) / 2.0;
    double rightSpeed = (xAxisSpeed + yAxisSpeed) / 2.0;

    Robot.climberWinch.setSpeed(leftSpeed, rightSpeed);
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

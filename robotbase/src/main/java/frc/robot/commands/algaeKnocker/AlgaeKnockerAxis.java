package frc.robot.commands.algaeKnocker;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AlgaeKnockerAxis extends Command {

  private Supplier<Double> m_axis;

  public AlgaeKnockerAxis(Supplier<Double> axis) {
    m_axis = axis;
    addRequirements(Robot.algaeKnocker);
  }

  @Override
  public void execute() {
    double axisValue = m_axis.get();
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

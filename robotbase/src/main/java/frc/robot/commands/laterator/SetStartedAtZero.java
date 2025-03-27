package frc.robot.commands.laterator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetStartedAtZero extends Command {

  @Override
  public void initialize() {
    Robot.laterator.setStartedAtZero(Robot.laterator.isAtZero());
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

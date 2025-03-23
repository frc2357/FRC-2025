package frc.robot.commands.laterator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetStartedAtZero extends Command {

  @Override
  public void initialize() {
    if (Robot.laterator.isAtZero()) {
      Robot.laterator.setStartedAtZero(true);
    } else {
      Robot.laterator.setStartedAtZero(false);
    }
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

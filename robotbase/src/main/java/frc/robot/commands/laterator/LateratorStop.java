package frc.robot.commands.laterator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LateratorStop extends Command {

  public LateratorStop() {
    addRequirements(Robot.laterator);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean wasInterupted) {
    Robot.laterator.stop();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

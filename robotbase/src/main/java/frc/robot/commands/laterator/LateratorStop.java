package frc.robot.commands.laterator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class LateratorStop extends Command {

  public LateratorStop() {
    addRequirements(Robot.laterator);
  }

  public boolean isFinished() {
    return true;
  }

  public void end(boolean wasInterupted) {
    Robot.laterator.stop();
  }
}

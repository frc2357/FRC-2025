package frc.robot.commands.climberWinch;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberWinchStop extends Command {

  public ClimberWinchStop() {
    addRequirements(Robot.climberWinch);
  }

  public boolean isFinished() {
    return true;
  }

  public void end(boolean wasInterupted) {
    Robot.climberWinch.stop();
  }
}

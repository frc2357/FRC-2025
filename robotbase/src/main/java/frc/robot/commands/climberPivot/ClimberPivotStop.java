package frc.robot.commands.climberPivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimberPivotStop extends Command {

  public ClimberPivotStop() {
    addRequirements(Robot.climberPivot);
  }

  public boolean isFinished() {
    return true;
  }

  public void end(boolean wasInterupted) {
    Robot.climberPivot.stop();
  }
}

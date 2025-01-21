package frc.robot.commands.algaePivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AlgaePivotStop extends Command {

  public AlgaePivotStop() {
    addRequirements(Robot.algaePivot);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.algaePivot.stop();
  }
}

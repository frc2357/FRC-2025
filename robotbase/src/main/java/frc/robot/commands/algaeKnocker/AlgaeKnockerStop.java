package frc.robot.commands.algaeKnocker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AlgaeKnockerStop extends Command {

  public AlgaeKnockerStop() {
    addRequirements(Robot.algaeKnocker);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.algaeKnocker.stop();
  }
}

package frc.robot.commands.coralRunner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class CoralRunnerStop extends Command {

  public CoralRunnerStop() {
    addRequirements(Robot.coralRunner);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean wasInterupted) {
    Robot.coralRunner.stop();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

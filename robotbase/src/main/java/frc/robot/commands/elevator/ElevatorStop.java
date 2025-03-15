package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorStop extends Command {

  public ElevatorStop() {
    addRequirements(Robot.elevator);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean wasInterupted) {
    Robot.elevator.stop();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}

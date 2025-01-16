package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorStop extends Command {

  public ElevatorStop() {
    addRequirements(Robot.elevator);
  }

  public boolean isFinished() {
    return true;
  }

  public void end(boolean wasInterupted) {
    Robot.elevator.stop();
  }
}

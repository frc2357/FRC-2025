package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorZero extends Command {

  public ElevatorZero() {
    addRequirements(Robot.elevator);
  }

  @Override
  public void initialize() {
    Robot.elevator.setSpeed(Constants.ELEVATOR.ZERO_SPEED);
  }

  @Override
  public boolean isFinished() {
    return Robot.elevator.isAtZero();
  }

  @Override
  public void end(boolean interrupted) {
    Robot.elevator.stop();
    if (!interrupted) {
      Robot.elevator.setZero();
    }
  }
}

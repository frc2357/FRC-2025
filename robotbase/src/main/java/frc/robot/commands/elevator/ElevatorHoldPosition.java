package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ElevatorHoldPosition extends Command {

  public ElevatorHoldPosition() {
    addRequirements(Robot.elevator);
  }

  @Override
  public void initialize() {
    Robot.elevator.setTargetRotations(Robot.elevator.getRotations());
  }
}

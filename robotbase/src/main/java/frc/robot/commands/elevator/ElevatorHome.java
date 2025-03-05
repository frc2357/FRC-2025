package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;

public class ElevatorHome extends SequentialCommandGroup {

  public ElevatorHome() {
    super(
      new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.HOME).until(() ->
        Robot.elevator.isAtZero()
      ),
      new ElevatorZero()
    );
  }
}

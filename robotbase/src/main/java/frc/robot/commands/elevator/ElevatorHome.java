package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ELEVATOR;

public class ElevatorHome extends SequentialCommandGroup {

  public ElevatorHome() {
    super(
      new ElevatorSetDistance(ELEVATOR.SETPOINTS.HOME).withDeadline(
        new WaitCommand(1)
      ),
      new ElevatorZero()
    );
  }
}

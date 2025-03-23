package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ELEVATOR;

public class ElevatorHome extends SequentialCommandGroup {

  public ElevatorHome() {
    super(new ElevatorSetDistance(ELEVATOR.SETPOINTS.HOME));
  }
}

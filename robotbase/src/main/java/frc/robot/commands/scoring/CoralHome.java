package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class CoralHome extends ParallelCommandGroup {

  public CoralHome() {
    super(
      new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.HOME),
      new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.HOME)
    );
  }
}

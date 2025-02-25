package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetRotations;

public class CoralHome extends ParallelCommandGroup {

  public CoralHome() {
    super(
      new LateratorSetRotations(Constants.LATERATOR.SETPOINTS.HOME),
      new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.HOME)
    );
  }
}

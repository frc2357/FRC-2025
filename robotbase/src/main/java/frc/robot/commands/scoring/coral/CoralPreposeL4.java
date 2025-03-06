package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;

public class CoralPreposeL4 extends ParallelCommandGroup {

  public CoralPreposeL4() {
    super(new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.L4_PREPOSE));
  }
}

package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetRotations;

public class CoralPreposeL3 extends ParallelCommandGroup {

  public CoralPreposeL3() {
    super(
      new LateratorSetRotations(Constants.LATERATOR.SETPOINTS.L3_PREPOSE),
      new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.L3_PREPOSE)
    );
  }
}

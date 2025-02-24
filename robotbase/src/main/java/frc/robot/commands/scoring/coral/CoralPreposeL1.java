package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class CoralPreposeL1 extends ParallelCommandGroup {

  public CoralPreposeL1() {
    super(
      new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.L1_PREPOSE),
      new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.L1_PREPOSE)
    );
  }
}

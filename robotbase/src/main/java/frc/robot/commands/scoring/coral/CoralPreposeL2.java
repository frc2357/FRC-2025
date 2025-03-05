package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class CoralPreposeL2 extends ParallelCommandGroup {

  public CoralPreposeL2() {
    super(new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.L2_PREPOSE));
  }
}

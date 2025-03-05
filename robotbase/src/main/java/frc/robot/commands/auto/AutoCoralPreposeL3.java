package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class AutoCoralPreposeL3 extends ParallelCommandGroup {

  public AutoCoralPreposeL3() {
    super(
      new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.L3_PREPOSE),
      new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.L3_PREPOSE)
    );
  }
}

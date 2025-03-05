package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class AutoCoralPreposeL4 extends ParallelCommandGroup {

  public AutoCoralPreposeL4() {
    super(
      new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.L4_PREPOSE),
      new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.L4_PREPOSE)
    );
  }
}

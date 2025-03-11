package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class AutoCoralPreposeL1 extends ParallelCommandGroup {

  public AutoCoralPreposeL1() {
    super(
      new LateratorSetDistance(() -> Constants.LATERATOR.SETPOINTS.L1_PREPOSE),
      new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.L1_PREPOSE)
    );
  }
}

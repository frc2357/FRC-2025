package frc.robot.commands.scoring.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class AutoCoralPreposeL2 extends SequentialCommandGroup {

  public AutoCoralPreposeL2() {
    super(
      new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.L2_PREPOSE),
      new LateratorSetDistance(
        Constants.LATERATOR.SETPOINTS.L2_PREPOSE
      ).withTimeout(0.3)
    );
  }
}

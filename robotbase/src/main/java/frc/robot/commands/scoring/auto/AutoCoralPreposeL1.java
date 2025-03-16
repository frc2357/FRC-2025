package frc.robot.commands.scoring.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class AutoCoralPreposeL1 extends SequentialCommandGroup {

  public AutoCoralPreposeL1() {
    super(
      new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.L1_PREPOSE),
      new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.L1_PREPOSE)
    );
  }
}

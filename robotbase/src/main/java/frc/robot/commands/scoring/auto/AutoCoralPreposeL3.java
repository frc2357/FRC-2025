package frc.robot.commands.scoring.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class AutoCoralPreposeL3 extends SequentialCommandGroup {

  public AutoCoralPreposeL3() {
    super(
      new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.L3_PREPOSE),
      new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.L3_PREPOSE)
    );
  }
}

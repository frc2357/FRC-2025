package frc.robot.commands.descoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.LATERATOR;
import frc.robot.commands.algaeKnocker.AlgaeKnockerSetSpeed;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class RemoveAlgaeLow extends ParallelCommandGroup {

  public RemoveAlgaeLow() {
    super(
      new ElevatorSetDistance(ELEVATOR.SETPOINTS.LOW_ALGAE),
      new LateratorSetDistance(LATERATOR.SETPOINTS.L3_PREPOSE),
      new AlgaeKnockerSetSpeed(-0.5)
    );
  }
}

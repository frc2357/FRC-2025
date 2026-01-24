package frc.robot.commands.descoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ELEVATOR;
import frc.robot.Constants.LATERATOR;
import frc.robot.commands.algaeKnocker.AlgaeKnockerSetSpeed;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;
import frc.robot.commands.util.PressToContinue;

public class RemoveAlgaeLow extends SequentialCommandGroup {

  public RemoveAlgaeLow(Trigger button) {
    super(
      new ElevatorSetDistance(ELEVATOR.SETPOINTS.LOW_ALGAE),
      (new LateratorSetDistance(LATERATOR.SETPOINTS.L3_PREPOSE)),
      new ParallelRaceGroup(
        (new AlgaeKnockerSetSpeed(-1)),
        (new PressToContinue(button))
      ),
      (new LateratorSetDistance(LATERATOR.SETPOINTS.HOME)),
      (new ElevatorSetDistance(ELEVATOR.SETPOINTS.HOME))
    );
  }
}

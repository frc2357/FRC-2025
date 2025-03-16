package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class CoralPreposeIntake extends ParallelCommandGroup {

  public CoralPreposeIntake() {
    super(
      new LateratorSetDistance(
        Constants.LATERATOR.SETPOINTS.INTAKE_PREPOSE
      ).alongWith(
        new WaitCommand(0.2).andThen(
          new ElevatorSetDistance(ELEVATOR.SETPOINTS.INTAKE_PREPOSE)
        )
      )
    );
  }
}

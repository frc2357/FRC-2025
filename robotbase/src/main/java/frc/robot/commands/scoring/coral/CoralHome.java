package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.laterator.LateratorSetDistance;
import frc.robot.commands.laterator.LateratorZero;

public class CoralHome extends ParallelCommandGroup {

  public CoralHome() {
    super(
      new LateratorSetDistance(Constants.LATERATOR.SETPOINTS.HOME),
      new LateratorZero()
      // new ElevatorSetDistance(Constants.ELEVATOR.SETPOINTS.HOME)
    );
  }
}

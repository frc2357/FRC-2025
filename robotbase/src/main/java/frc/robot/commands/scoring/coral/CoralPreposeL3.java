package frc.robot.commands.scoring.coral;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;

public class CoralPreposeL3 extends ParallelCommandGroup {

  public CoralPreposeL3() {
    super(
      new ElevatorSetDistance(
        Constants.ELEVATOR.SETPOINTS.L3_PREPOSE.plus(
          Units.Inches.of(
            SmartDashboard.getNumber("Elevator Setpoint Modifier", 0)
          )
        )
      )
    );
  }
}

package frc.robot.commands.scoring.coral;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;

public class CoralPreposeL1 extends SequentialCommandGroup {

  public CoralPreposeL1() {
    super(
      new ElevatorSetDistance(
        Constants.ELEVATOR.SETPOINTS.L1_PREPOSE.plus(
          Units.Inches.of(
            SmartDashboard.getNumber("Elevator Setpoint Modifier", 0)
          )
        )
      )
    );
  }
}

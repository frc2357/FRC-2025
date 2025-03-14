package frc.robot.commands.scoring.coral;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.LATERATOR;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;

public class CoralPreposeL1 extends SequentialCommandGroup {

  public CoralPreposeL1() {
    super(
      new LateratorSetDistance(LATERATOR.SETPOINTS.HOME),
      new ElevatorSetDistance(
        Constants.ELEVATOR.SETPOINTS.L1_PREPOSE.plus(
          Units.Inches.of(
            SmartDashboard.getNumber("Elevator Setpoint Modifier", 0)
          )
        )
      ),
      new LateratorSetDistance(LATERATOR.SETPOINTS.L1_PREPOSE)
    );
  }
}

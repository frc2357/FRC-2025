package frc.robot.commands.scoring;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.elevator.ElevatorSetDistance;
import frc.robot.commands.laterator.LateratorSetDistance;
import frc.robot.controls.controllers.ButtonboardController.ScoringLevel;

public class CoralPreposeScore extends ParallelCommandGroup {

  private Distance m_lateratorDistance;
  private Distance m_elevatorDistance;

  public CoralPreposeScore(ScoringLevel m_level) {
    super();
    switch (m_level) {
      case L1:
        m_elevatorDistance = Constants.ELEVATOR.SETPOINTS.L1_PREPOSE;
        m_lateratorDistance = Constants.LATERATOR.SETPOINTS.L1_PREPOSE;
      case L2:
        m_elevatorDistance = Constants.ELEVATOR.SETPOINTS.L2_PREPOSE;
        m_lateratorDistance = Constants.LATERATOR.SETPOINTS.L2_PREPOSE;
      case L3:
        m_elevatorDistance = Constants.ELEVATOR.SETPOINTS.L3_PREPOSE;
        m_lateratorDistance = Constants.LATERATOR.SETPOINTS.L3_PREPOSE;
      case L4:
        m_elevatorDistance = Constants.ELEVATOR.SETPOINTS.L4_PREPOSE;
        m_lateratorDistance = Constants.LATERATOR.SETPOINTS.L4_PREPOSE;
      default:
        break;
    }

    addCommands(
      new LateratorSetDistance(m_lateratorDistance),
      new ElevatorSetDistance(m_elevatorDistance)
    );
  }
}

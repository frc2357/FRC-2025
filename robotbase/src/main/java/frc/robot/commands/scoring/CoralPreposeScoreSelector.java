package frc.robot.commands.scoring;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.controls.controllers.ButtonboardController.ScoringLevel;

public class CoralPreposeScore {


 private Scoring PreposeSelector select(){
  return ScoringLevel.L1;
 }

  private final Command m_selectCommand = 
    new SelectCommand<>(
      Map.ofEntries(
        Map.entry(ScoringLevel.L1, new CoralPreposeL1()),
        Map.entry(ScoringLevel.L2, new CoralPreposeL2()),
        Map.entry(ScoringLevel.L3, new CoralPreposeL3()),
        Map.entry(ScoringLevel.L4, new CoralPreposeL4())),
      this::select);
    
  
}













/**
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
*/

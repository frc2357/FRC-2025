package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.controls.controllers.ButtonboardController.ScoringLevel;
import java.util.Map;

public class CoralHumanPrepose {

  private ScoringLevel[] m_levels = new ScoringLevel[] {
    ScoringLevel.None,
    ScoringLevel.L1,
    ScoringLevel.L2,
    ScoringLevel.L3,
    ScoringLevel.L4,
  };

  private int m_scoringPositions = m_levels.length;

  private int m_currentLevel = 0;

  private SelectCommand<ScoringLevel> m_command = new SelectCommand<>(
    Map.ofEntries(
      Map.entry(ScoringLevel.L1, new CoralPreposeL1()),
      Map.entry(ScoringLevel.L2, new CoralPreposeL2()),
      Map.entry(ScoringLevel.L3, new CoralPreposeL3()),
      Map.entry(ScoringLevel.L4, new CoralPreposeL4()),
      Map.entry(ScoringLevel.None, new CoralHome())
    ),
    this::supplier
  );

  public SelectCommand<ScoringLevel> getSelectCommand() {
    return m_command;
  }

  private ScoringLevel supplier() {
    m_currentLevel = m_currentLevel == m_scoringPositions - 1
      ? 0
      : m_currentLevel + 1;
    return m_levels[m_currentLevel];
  }

  public Command reset() {
    return new InstantCommand(() -> m_currentLevel = 0);
  }
}

package frc.robot.commands.scoring.coral;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LATERATOR;
import frc.robot.Robot;
import frc.robot.commands.intake.CoralIntake;
import frc.robot.controls.controllers.CommandButtonboardController.ScoringLevel;
import java.util.Map;
import javax.sound.midi.Sequencer;

public class CoralChooser {

  private ScoringLevel[] m_levels = new ScoringLevel[] {
    ScoringLevel.None,
    ScoringLevel.L1,
    ScoringLevel.L2,
    ScoringLevel.L3,
    ScoringLevel.L4,
  };

  private int m_scoringPositions = m_levels.length;

  private int m_currentLevel = 0;

  private SelectCommand<ScoringLevel> m_elevatorPreposeCommand =
    new SelectCommand<>(
      Map.ofEntries(
        Map.entry(ScoringLevel.L1, new CoralPreposeL1()),
        Map.entry(ScoringLevel.L2, new CoralPreposeL2()),
        Map.entry(ScoringLevel.L3, new CoralPreposeL3()),
        Map.entry(ScoringLevel.L4, new CoralPreposeL4()),
        Map.entry(ScoringLevel.None, new CoralHome())
      ),
      this::levelSupplier
    );

  public SelectCommand<ScoringLevel> getElevatorPreposeCommand() {
    return m_elevatorPreposeCommand;
  }

  private ScoringLevel levelSupplier() {
    m_currentLevel = m_currentLevel == m_scoringPositions - 1
      ? 0
      : m_currentLevel + 1;
    return m_levels[m_currentLevel];
  }

  public Distance getLateratorDistance() {
    switch (m_levels[m_currentLevel]) {
      case L1:
        return LATERATOR.SETPOINTS.L1_PREPOSE;
      case L2:
        return LATERATOR.SETPOINTS.L2_PREPOSE;
      case L3:
        return LATERATOR.SETPOINTS.L3_PREPOSE;
      case L4:
        return LATERATOR.SETPOINTS.L4_PREPOSE;
      default:
        return LATERATOR.SETPOINTS.HOME;
    }
  }

  public void resetLevel() {
    m_currentLevel = 0;
  }

  public SequentialCommandGroup selectL4() {
    // Set the current level to L3, then when the prepose command runs, it will go to L4 and set current level to L4
    return new InstantCommand(() -> m_currentLevel = 3).andThen(
      new CoralPreposeL4()
    );
  }

  private SelectCommand<Boolean> m_scoreCommand = new SelectCommand<Boolean>(
    Map.ofEntries(
      Map.entry(
        true,
        new CoralScore(() -> getLateratorDistance()).finallyDo(() ->
          resetLevel()
        )
      ),
      Map.entry(false, new CoralIntake())
    ),
    this::supplier
  );

  public SelectCommand<Boolean> getScoreCommand() {
    return m_scoreCommand;
  }

  private boolean supplier() {
    return Robot.coralRunner.isOuttakeBeamBroken();
  }
}

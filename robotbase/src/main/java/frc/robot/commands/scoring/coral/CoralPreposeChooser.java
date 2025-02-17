package frc.robot.commands.scoring.coral;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Robot;
import frc.robot.controls.controllers.CommandButtonboardController.ScoringLevel;
import java.util.Map;

public class CoralPreposeChooser extends SelectCommand<ScoringLevel> {

  public CoralPreposeChooser() {
    super(
      Map.ofEntries(
        Map.entry(ScoringLevel.L1, new CoralPreposeL1()),
        Map.entry(ScoringLevel.L2, new CoralPreposeL2()),
        Map.entry(ScoringLevel.L3, new CoralPreposeL3()),
        Map.entry(ScoringLevel.L4, new CoralPreposeL4()),
        Map.entry(ScoringLevel.None, new InstantCommand())
      ),
      Robot.buttonboard::getSelectedScoringLevel
    );
  }
}

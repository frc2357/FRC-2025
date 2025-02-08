package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Robot;
import frc.robot.controls.controllers.ButtonboardController.ScoringLevel;
import java.util.Map;

public class CoralPreposeChooser extends SelectCommand<ScoringLevel> {

  public CoralPreposeChooser() {
    super(
      Map.ofEntries(
        Map.entry(ScoringLevel.L1, new CoralPreposeL1()),
        Map.entry(ScoringLevel.L2, new CoralPreposeL2()),
        Map.entry(ScoringLevel.L3, new CoralPreposeL3()),
        Map.entry(ScoringLevel.L4, new CoralPreposeL4())
      ),
      Robot.buttonboard::getSelectedScoringLevel
    );
  }
}

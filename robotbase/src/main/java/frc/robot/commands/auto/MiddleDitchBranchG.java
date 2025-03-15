package frc.robot.commands.auto;

import frc.robot.Constants;
import frc.robot.commands.scoring.CoralHome;
import frc.robot.commands.scoring.auto.AutoCoralConfirmScore;
import frc.robot.commands.scoring.auto.AutoCoralPreposeL4;

public class MiddleDitchBranchG extends AutoBase {

  public MiddleDitchBranchG() {
    super("Mid Branch G | 1P | None", "MiddleDitchBranchG");
    m_startTraj
      .done()
      .onTrue(
        new AutoCoralPreposeL4()
          .andThen(
            new AutoCoralConfirmScore(
              Constants.CORAL_RUNNER.SCORING_PERCENT_L4
            ),
            new CoralHome()
          )
      );
  }
}

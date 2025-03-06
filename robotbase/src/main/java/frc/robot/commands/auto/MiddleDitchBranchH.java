package frc.robot.commands.auto;

import frc.robot.Constants.LATERATOR;
import frc.robot.commands.scoring.coral.CoralHome;
import frc.robot.commands.scoring.coral.CoralPreposeL4;
import frc.robot.commands.scoring.coral.CoralScore;

public class MiddleDitchBranchH extends AutoBase {

  public MiddleDitchBranchH() {
    super("Mid Branch H | 1P | None", "MiddleDitchBranchH");
    m_startTraj
      .done()
      .onTrue(
        new CoralPreposeL4()
          .andThen(
            new CoralScore(() -> LATERATOR.SETPOINTS.L4_PREPOSE),
            new CoralHome()
          )
      );
  }
}

package frc.robot.commands.auto;

import static frc.robot.Constants.CHOREO.*;

import choreo.auto.AutoTrajectory;
import frc.robot.Constants;
import frc.robot.commands.intake.CoralIntake;
import frc.robot.commands.intake.CoralPreposeIntake;
import frc.robot.commands.intake.CoralRetract;
import frc.robot.commands.scoring.auto.AutoCoralConfirmScore;
import frc.robot.commands.scoring.auto.AutoCoralPreposeL4;

public class CL33Peice extends AutoBase {

  public CL33Peice() {
    super("Cage Left 3 | 2 P", "CB3ToBranchJ");
    // all the commented out code should work the same, but we dont know until we tune the path, so its here for now.
    // makeAutoFromSegments(
    //   "branchJToBlueS",
    //   "BlueSToBranchK",
    //   "branchKToBlueS",
    //   "BlueSToBranchL",
    //   "branchLToBlueS",
    //   "BlueSToBranchI",
    //   "branchIToBlueS"
    // );
    AutoTrajectory branchJToBlueS = m_routine.trajectory("branchJToBlueS");
    AutoTrajectory BlueSToBranchK = m_routine.trajectory("BlueSToBranchK");
    AutoTrajectory branchKToBlueS = m_routine.trajectory("branchKToBlueS");
    // AutoTrajectory BlueSToBranchL = m_routine.trajectory("BlueSToBranchL");
    // AutoTrajectory branchLToBlueS = m_routine.trajectory("branchLToBlueS");
    // AutoTrajectory BlueSToBranchI = m_routine.trajectory("BlueSToBranchI");
    // AutoTrajectory branchIToBlueS = m_routine.trajectory("branchIToBlueS");
    // This is with everything manually put into segments
    // // shoved down into segments, to make things really small
    // scoringSegment(m_startTraj, branchJToBlueS);
    // intakingSegment(branchJToBlueS, BlueSToBranchK);
    // scoringSegment(BlueSToBranchK, branchKToBlueS);
    // intakingSegment(branchKToBlueS, BlueSToBranchL);
    // scoringSegment(BlueSToBranchL, branchLToBlueS);
    // intakingSegment(branchLToBlueS, BlueSToBranchI);
    // scoringSegment(BlueSToBranchI, branchIToBlueS);

    // as we get close to branch J we prepose to score
    // m_startTraj.atTimeBeforeEnd(PREPOSE_SECONDS).onTrue(new CoralPreposeL4());
    // when at branch J we score, lower the elevator, and move on
    m_startTraj
      .done()
      .onTrue(
        new AutoCoralPreposeL4()
          .andThen(
            new AutoCoralConfirmScore(
              Constants.CORAL_RUNNER.SCORING_PERCENT_L4
            ),
            new CoralPreposeIntake(),
            branchJToBlueS.cmd()
          )
      ); //score coral 1

    // when at the coral station, we intake coral and then go to the next branch
    branchJToBlueS
      .done()
      .onTrue(
        new CoralIntake()
          .andThen(new CoralRetract())
          .andThen(BlueSToBranchK.cmd())
      );
    // as we get close to the branch, we prepose to score
    BlueSToBranchK.atTimeBeforeEnd(PREPOSE_SECONDS).onTrue(
      new AutoCoralPreposeL4()
    );
    // when at the branch, we score and then move back to the station
    BlueSToBranchK.done()
      .onTrue(
        new AutoCoralPreposeL4()
          .andThen(
            new AutoCoralConfirmScore(Constants.CORAL_RUNNER.SCORING_PERCENT_L4) // score coral 2
              .andThen(new CoralPreposeIntake())
              .andThen(branchKToBlueS.cmd())
          )
      );
    // when at the coral station, we intake coral and then go to the next branch
    branchKToBlueS
      .done()
      .onTrue(
        new CoralIntake().andThen(new CoralRetract())
        /*.andThen(BlueSToBranchL.cmd()) */
      );
    // BlueSToBranchL.atTimeBeforeEnd(PREPOSE_SECONDS).onTrue(
    //   new AutoCoralPreposeL4()
    // );
    // BlueSToBranchL.done()
    //   .onTrue(
    //     new AutoCoralConfirmScore(
    //       Constants.CORAL_RUNNER.SCORING_PERCENT_L4
    //     ).andThen(new CoralPreposeIntake(), branchLToBlueS.cmd()) // score coral 3
    //   );
    // // when at the coral station, we intake coral and then go to the next branch
    // branchLToBlueS
    //   .done()
    //   .onTrue(new CoralIntake().andThen(new CoralRetract()).andThen(BlueSToBranchI.cmd()));
    // BlueSToBranchI.atTimeBeforeEnd(PREPOSE_SECONDS).onTrue(
    //   new AutoCoralPreposeL4()
    // );
    // BlueSToBranchI.done()
    //   .onTrue(
    //     new AutoCoralConfirmScore(
    //       Constants.CORAL_RUNNER.SCORING_PERCENT_L4 // score coral 4
    //     ).andThen(new CoralPreposeIntake(), branchIToBlueS.cmd()) // score coral 4
    //   );
    // // when at the coral station, we intake coral, and wait for auto to end (if it hasnt already)
    // branchIToBlueS.done().onTrue(new CoralIntake().andThen(new CoralRetract()));
  }
}

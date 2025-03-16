package frc.robot.commands.auto;

import static frc.robot.Constants.CHOREO.*;

import choreo.auto.AutoTrajectory;
import frc.robot.Constants;
import frc.robot.commands.intake.CoralIntake;
import frc.robot.commands.intake.CoralPreposeIntake;
import frc.robot.commands.intake.CoralRetract;
import frc.robot.commands.intake.CoralSettle;
import frc.robot.commands.scoring.CoralHome;
import frc.robot.commands.scoring.auto.AutoCoralConfirmScore;
import frc.robot.commands.scoring.auto.AutoCoralPreposeL4;

public class C3R3Peice extends AutoBase {

  public C3R3Peice() {
    super("Cage Right 3 | 3 P", "CR3ToBranchE");
    // all the commented out code should work the same, but we dont know until we tune the path, so its here for now.
    // makeAutoFromSegments(
    //   "branchJToRedS",
    //   "RedSToBranchK",
    //   "branchKToRedS",
    //   "RedSToBranchL",
    //   "branchLToRedS",
    //   "RedSToBranchI",
    //   "branchIToRedS"
    // );
    AutoTrajectory branchEToRedS = m_routine.trajectory("branchEToRedS");
    AutoTrajectory redSToBranchD = m_routine.trajectory("RedSToBranchD");
    AutoTrajectory branchDToRedS = m_routine.trajectory("branchDToRedS");
    AutoTrajectory redSToBranchC = m_routine.trajectory("RedSToBranchC");
    AutoTrajectory branchCToRedS = m_routine.trajectory("branchCToRedS");
    // AutoTrajectory RedSToBranchI = m_routine.trajectory("RedSToBranchI");
    // AutoTrajectory branchIToRedS = m_routine.trajectory("branchIToRedS");
    // This is with everything manually put into segments
    // // shoved down into segments, to make things really small
    // scoringSegment(m_startTraj, branchJToRedS);
    // intakingSegment(branchJToRedS, RedSToBranchK);
    // scoringSegment(RedSToBranchK, branchKToRedS);
    // intakingSegment(branchKToRedS, RedSToBranchL);
    // scoringSegment(RedSToBranchL, branchLToRedS);
    // intakingSegment(branchLToRedS, RedSToBranchI);
    // scoringSegment(RedSToBranchI, branchIToRedS);

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
            new CoralPreposeIntake().andThen(branchEToRedS.cmd())
          )
      ); //score coral 1

    // when at the coral station, we intake coral and then go to the next branch
    branchEToRedS
      .done()
      .onTrue(
        new CoralIntake()
          .andThen(
            new CoralHome(() -> false).andThen(
              redSToBranchD.cmd().alongWith(new CoralSettle())
            )
          )
      );
    // when at the branch, we score and then move back to the station
    redSToBranchD
      .done()
      .onTrue(
        new AutoCoralPreposeL4()
          .andThen(
            new AutoCoralConfirmScore(
              Constants.CORAL_RUNNER.SCORING_PERCENT_L4
            ).andThen(new CoralPreposeIntake(), branchDToRedS.cmd())
          ) // score coral 2
      );
    // when at the coral station, we intake coral and then go to the next branch
    branchDToRedS
      .done()
      .onTrue(
        new CoralIntake()
          .andThen(
            new CoralHome(() -> false),
            redSToBranchC.cmd().alongWith(new CoralSettle())
          )
      );
    redSToBranchC
      .done()
      .onTrue(
        new AutoCoralPreposeL4()
          .andThen(
            new AutoCoralConfirmScore(
              Constants.CORAL_RUNNER.SCORING_PERCENT_L4
            ).andThen(new CoralPreposeIntake(), branchCToRedS.cmd()) // score coral 3
          )
      );
    // when at the coral station, we intake coral and then go to the next branch
    branchCToRedS
      .done()
      .onTrue(
        new CoralIntake()
          .andThen(new CoralRetract(false))/* .andThen(RedSToBranchI.cmd())*/
      );
    // RedSToBranchI.atTimeBeforeEnd(PREPOSE_SECONDS).onTrue(
    //   new AutoCoralPreposeL4()
    // );
    // RedSToBranchI.done()
    //   .onTrue(
    //     new AutoCoralConfirmScore(
    //       Constants.CORAL_RUNNER.SCORING_PERCENT_L4 // score coral 4
    //     ).andThen(new CoralPreposeIntake(), branchIToRedS.cmd()) // score coral 4
    //   );
    // // when at the coral station, we intake coral, and wait for auto to end (if it hasnt already)
    // branchIToRedS.done().onTrue(new CoralIntake().andThen(new CoralRetract()));
  }
}

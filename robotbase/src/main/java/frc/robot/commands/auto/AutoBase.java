package frc.robot.commands.auto;

import static frc.robot.Constants.CHOREO.*;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Robot;
import frc.robot.commands.intake.CoralIntake;
import frc.robot.commands.intake.CoralPreposeIntake;
import frc.robot.commands.scoring.coral.CoralPreposeL4;
import frc.robot.commands.scoring.coral.CoralScore;
import frc.robot.commands.util.VariableWaitCommand;

public class AutoBase {

  protected AutoRoutine m_routine;
  protected AutoTrajectory m_startTraj;
  private String m_name;

  /**
   * This will initialize an auto routine
   * Will create the first trajectory, and set the routine to wait, reset odometry, and run the first trajectory
   * @param name Name of the auto routine
   */
  public AutoBase(String name) {
    this(name, name);
  }

  /**
   * This will initialize an auto routine
   * Will create the first trajectory, and set the routine to wait, reset odometry, and run the first trajectory
   * @param name Name of the auto routine
   * @param startTraj Trajectory to start with
   */
  public AutoBase(String name, String startTraj) {
    m_name = name;
    // This is how you make an auto routine. The name should be essentially the same as the name of the function it is in.
    m_routine = AUTO_FACTORY.newRoutine(m_name);

    // This is how you make a trajectory to put into the AutoRoutine. The trajectoryName is the name of the file in Choreo.
    // Any deviation from that name will result in the file not being found.
    m_startTraj = m_routine.trajectory(startTraj);
    // This is how you reset the odometry and make the routine use a trajectory. This is a veyr regular thing that you will have to do.
    m_routine
      // .active() returns a trigger that is true while the AutoRoutine is running
      .active()
      // .onTrue() lets us run commands once that trigger becomes true.
      .onTrue(
        // Commands.sequence() lets us sequence commands to run, letting us run multiple commands per trigger.
        Commands.sequence(
          new VariableWaitCommand(() ->
            SmartDashboard.getNumber("wait seconds", 0)
          ),
          // This command resets the odometry, and it MUST be run on the starting traj, or very bad things will happen.
          m_startTraj.resetOdometry(),
          // This runs the trajectory that was loaded earlier. This is needed to make the AutoRoutine actually run the trajectory, instead of doing nothing.
          m_startTraj.cmd()
        )
      );
  }

  /**
   * Takes in 2 AutoTrajectory objects and sets up the requiste triggers to go from the coral station to the reef (traj1),
   * score, and start heading back to the coral station (traj2)
   * @param traj1 The AutoTrajectory that takes the robot from the coral sttation to the reef
   * @param traj2 The AutoTrajectory that takes the robot from the reef to the coral station
   */
  protected void scoringSegment(AutoTrajectory traj1, AutoTrajectory traj2) {
    traj1.atTimeBeforeEnd(PREPOSE_SECONDS).onTrue(new CoralPreposeL4());
    traj1
      .done()
      .onTrue(new CoralScore().andThen(new CoralPreposeIntake(), traj2.cmd()));
  }

  /**
   * Takes in 2 AutoTrajectory objects and sets up the requiste triggers to go from the coral station to the reef (traj1),
   * score, and start heading back to the coral station (traj2)
   * @param traj1 The AutoTrajectory that takes the robot from the coral sttation to the reef
   * @param traj2 The AutoTrajectory that takes the robot from the reef to the coral station
   */
  protected void scoringSegment(String traj1, String traj2) {
    scoringSegment(m_routine.trajectory(traj1), m_routine.trajectory(traj2));
  }

  /**
   * Takes in 2 AutoTrajectory objects and sets up the requiste triggers to make the robot go from the reef to the coral station (traj1),
   * intake, and go back to the reef (traj2)
   * @param traj1 The AutoTrajectory that takes the robot from the reef to the coral station
   * @param traj2 The AutoTrajectory that takes the robot from the coral station back to the reef
   */
  protected void intakingSegment(AutoTrajectory traj1, AutoTrajectory traj2) {
    traj1.done().onTrue(new CoralIntake().andThen(traj2.cmd()));
  }

  /**
   * Takes in 2 AutoTrajectory objects and sets up the requiste triggers to make the robot go from the reef to the coral station (traj1),
   * intake, and go back to the reef (traj2)
   * @param traj1 The AutoTrajectory that takes the robot from the reef to the coral station
   * @param traj2 The AutoTrajectory that takes the robot from the coral station back to the reef
   */
  protected void intakingSegment(String traj1, String traj2) {
    intakingSegment(m_routine.trajectory(traj1), m_routine.trajectory(traj2));
  }

  /**
   * Takes an arbitrary number of trajectory names and sets up the required triggers to make them run a full auto
   * @param trajectoryNames The list of trajectories that you want to make into a full auto, without the starting trajectory.
   */
  protected void makeAutoFromSegments(String... trajectoryNames) {
    // we start by making a scoring segment with the starting trajectory, and the first trajectory in the list
    scoringSegment(m_startTraj, m_routine.trajectory(trajectoryNames[0]));
    // we then start looping through the provided trajectory names to segment them based on if i is even or odd
    for (int i = 0; i < trajectoryNames.length - 1; i++) {
      // we score with the start trajectory, so 0 is going to the coral station, making it an intaking segment.
      if (i % 2 == 0) {
        intakingSegment(trajectoryNames[i], trajectoryNames[i + 1]);
      } else { // segments switch between intaking and scoring
        scoringSegment(trajectoryNames[i], trajectoryNames[i + 1]);
      }
    }
  }

  public AutoRoutine getRoutine() {
    return m_routine;
  }

  @Override
  public String toString() {
    return m_name;
  }
}

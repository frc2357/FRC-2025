package frc.robot.commands.auto;

import static frc.robot.Constants.CHOREO.AUTO_FACTORY;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
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
          // This command resets the odometry, and it MUST be run on every path, or very bad things will happen.
          m_startTraj.resetOdometry(),
          // This runs the trajectory that was loaded earlier. This is needed to make the AutoRoutine actually run the trajectory, instead of doing nothing.
          m_startTraj.cmd()
        )
      );
  }

  public AutoRoutine getRoutine() {
    return m_routine;
  }

  @Override
  public String toString() {
    return m_name;
  }
}

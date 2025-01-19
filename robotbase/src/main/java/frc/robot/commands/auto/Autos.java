package frc.robot.commands.auto;

import static frc.robot.Constants.CHOREO.AUTO_FACTORY;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.util.VariableWaitCommand;

public class Autos {

  public Autos() {}

  /**
   * Resets the odometry for the start of an auto. This MUST be used ONLY ONCE, at the start of each auto.
   * Puts the command to reset the odometry into a trigger for the start of the provided AutoRoutine.
   * @param trajectoryName The name of the trajectory to reset the odometry to the start of.
   * @param routine The AutoRoutine that you are going to reset the odometry to.
   * @return The trigger for the start of the AutoRoutine.
   */
  private Command resetOdometryCommand(String trajectoryName) {
    return Commands.sequence(
      AUTO_FACTORY.resetOdometry(trajectoryName),
      new VariableWaitCommand(() -> SmartDashboard.getNumber("wait seconds", 0))
    );
  }

  // This is an example of how you make an auto. it must be in a function, in this file, which returns an AutoRoutine object.
  public AutoRoutine cubeTestPath() {
    // This is how you make an auto routine. The name should be essentially the same as the name of the function it is in.
    AutoRoutine routine = AUTO_FACTORY.newRoutine("CubeTestPath");

    // This is how you make a trajectory to put into the AutoRoutine. The trajectoryName is the name of the file in Choreo.
    // Any deviation from that name will result in the file not being found.
    AutoTrajectory cubeTestPathTraj = routine.trajectory("CubeTestPath");

    // This is how you reset the odometry and make the routine use a trajectory. This is a veyr regular thing that you will have to do.
    routine
      // .active() returns a trigger that is true while the AutoRoutine is running
      .active()
      // .onTrue() lets us run commands once that trigger becomes true.
      .onTrue(
        // Commands.sequence() lets us sequence commands to run, letting us run multiple commands per trigger.
        Commands.sequence(
          // This command resets the odometry, and it MUST be run on every path, or very bad things will happen.
          resetOdometryCommand("CubeTestPath"),
          // This runs the trajectory that was loaded earlier. This is needed to make the AutoRoutine actually run the trajectory, instead of doing nothing.
          cubeTestPathTraj.cmd()
        )
      );

    return routine;
  }
}

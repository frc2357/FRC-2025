package frc.robot.commands.drive;

import java.time.Instant;

import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CHOREO;
import frc.robot.Robot;

public class DriveChoreoPath extends SequentialCommandGroup {

  private String m_pathName; // The name of the path file
  private AutoTrajectory m_traj; // The generated trajectory object
  private TrajectorySample<SwerveSample> m_startingState; // The starting state of the robot
  private Pose2d m_startingPose; // The starting pose of the robot
  private boolean m_targetLock; // Whether or not this specific path should target lock or not
  private int m_splitIndex;

  /**
   * A utility command to start a Choreo path without using the Trigger API.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   */
  public DriveChoreoPath(String trajectoryFileName) {
    // Overloaded constructor, sets the gyro yaw to zero and pose x, y to starting
    // position
    this(trajectoryFileName, 0);
  }

  /**
   * A utility command to start a Choreo path without using the Trigger API.
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   */
  public DriveChoreoPath(String trajectoryFileName, int splitIndex) {
    // Overloaded constructor, sets the gyro yaw to zero and pose x, y to starting
    // position
    this(trajectoryFileName, splitIndex, false);
  }

  /**
   * A utility command to start a Choreo path without using the Trigger API.
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   */
  public DriveChoreoPath(String trajectoryFileName, int splitIndex, boolean setPoseToStartTrajectory) {
    // Overloaded constructor, sets the gyro yaw to zero and pose x, y to starting
    // position
    this(trajectoryFileName, 0, setPoseToStartTrajectory, false);
  }

  /**
   * A utility command to start a Choreo path without using the Trigger API.<p>
   * You still need to use {@link #noTriggers()} or {@link #withTriggers()} to actually run the path, due to
   * how we handle Choreo
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   * @param splitIndex The split of the path to use. Must be at least 0.
   * @param setPoseToStartTrajectory Whether or not to set the robot pose to the paths starting
   *     trajectory.
   */
  public DriveChoreoPath(
      String trajectoryFileName, int splitIndex, boolean setPoseToStartTrajectory, boolean targetLock) {
    m_targetLock = targetLock;
    m_splitIndex = splitIndex;
    m_pathName = trajectoryFileName.contains(".") ? trajectoryFileName.split(".")[0] : trajectoryFileName;
    // m_traj = CHOREO.AUTO_FACTORY.trajectory(m_pathName, CHOREO.AUTO_LOOP);
    addCommands(
        new InstantCommand(
          () -> {
            // m_startingState = m_traj.getInitialSample(CHOREO.CHOREO_AUTO_MIRROR_PATHS.getAsBoolean()).get();
            // m_startingPose = m_startingState.getPose();
            m_startingPose = m_traj.getInitialPose().get();
          }
        ));
    
    // Set the gyro yaw to 0 and the pose x, y to the starting position of the path
    if (setPoseToStartTrajectory) {
      addCommands(new InstantCommand(() -> {
        System.out.println("[DriveChoreoPath] SETTING POSE TO: " + m_startingPose);
        Robot.swerve.setPose2d(m_startingPose);
      }));
    }
    
    // addCommands(
    //     // Set the drive velocity x, y and angular velocity to the starting state's
    //     // number
    //     // This should help the wheels "straighten" up before starting the path
    //     new InstantCommand(
    //         () ->
    //             Robot.swerve.driveFieldRelative(
    //                 m_startingState.getChassisSpeeds().vxMetersPerSecond,
    //                 m_startingState.getChassisSpeeds().vyMetersPerSecond,
    //                 m_startingState.getChassisSpeeds().omegaRadiansPerSecond))
    // );

    
    // addCommands(new InstantCommand(() -> {
    //   System.out.println("[DriveChoreoPath] CURRENT POSE V STARTING POSE: " + Robot.swerve.getPose2d() + " | " + m_startingPose);
    // }));
    // addCommands( 
    // new InstantCommand(
    //   () -> System.out.println("[DriveChoreoPath] RUNNING PATH: " + m_pathName)),
    //   CHOREO.AUTO_FACTORY.trajectoryCommand(m_pathName)
    // );
  }

    // /**
    // * This runs the path with no triggers, and should NOT allow use of the triggers API in other sections of the path. \n
    // * You should use triggers. They are cleaner.
    // */
    // public SequentialCommandGroup noTriggers(){
    //     addCommands(new InstantCommand(() -> {
    //       System.out.println("[DriveChoreoPath] CURRENT POSE V STARTING POSE: " + Robot.swerve.getPose2d() + " | " + m_startingPose);
    //     }));
    //     addCommands( 
          
    //     new InstantCommand(
    //       () -> System.out.println("[DriveChoreoPath] RUNNING PATH: " + m_pathName)),
    //       CHOREO.AUTO_FACTORY.trajectoryCommand(m_pathName)
    //   );
    //   return this; // returns the DriveChoreoPath SequentialCommandGroup
    // }

    // public AutoTrajectory withTriggers(){ 
    //   addCommands(new InstantCommand(
    //     () -> System.out.println("[DriveChoreoPath] RUNNING PATH: " + m_pathName)));
    //   var autoTraj = CHOREO.AUTO_FACTORY.trajectory(m_pathName, m_splitIndex, CHOREO.AUTO_FACTORY.newLoop(m_pathName));
    //   autoTraj.atTime(0).onTrue(this); // adds the DriveChoreoPath SequentialCommandGroup
    //     // as a trigger to the start of the AutoTrajectory via triggers. no clue if this works.
    //   return autoTraj;
    // }

  @Override
  public String toString() {
    return m_pathName;
  }
}

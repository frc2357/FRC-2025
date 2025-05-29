package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.DRIVE_TO_POSE.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.util.Utility;

public class DriveToPoseHandler extends Command {

  public enum RouteAroundReef {
    Clockwise,
    CounterClockwise,
    Fastest,
    None,
  }

  protected Pose2d m_currPose, m_currentTarget, m_currentToldTarget, m_finalGoal;

  protected DriveToPose m_currDriveToPose;

  protected RouteAroundReef m_routeAroundReef;

  protected Command m_finalApproachCommand;

  public DriveToPoseHandler() {
    this(RouteAroundReef.None);
  }

  public DriveToPoseHandler(RouteAroundReef routeAroundReef) {
    this(routeAroundReef, null);
  }

  public DriveToPoseHandler(
    RouteAroundReef routeAroundReef,
    Command finalApproachCommand
  ) {
    m_routeAroundReef = routeAroundReef;
    m_finalApproachCommand = finalApproachCommand;
  }

  @Override
  public void initialize() {
    m_currentTarget = Robot.swerve.getFieldRelativePose2d();
    m_currentToldTarget = m_currentTarget;
    m_currPose = Robot.swerve.getFieldRelativePose2d();
    m_currDriveToPose = new DriveToPose((Pose2d currentPose) ->
      getNewTarget(m_currentTarget, currentPose)
    ); // make a DriveToPose that we have control of
    m_currDriveToPose.schedule();
  }

  @Override
  public void execute() {
    m_currPose = Robot.swerve.getFieldRelativePose2d();
  }

  @Override
  public boolean isFinished() {
    return isAtTarget(m_finalGoal, m_currPose, FINAL_APPROACH_TOLERANCE_POSE);
  }

  @Override
  public void end(boolean isInteruptted) {
    m_currDriveToPose.cancel();
    Robot.swerve.stopMotors();
  }

  protected boolean isAtTarget(
    Pose2d targetPose,
    Pose2d currentPose,
    Pose2d tolerancePose
  ) {
    return Utility.isWithinTolerance(
      currentPose.getTranslation(),
      targetPose.getTranslation(),
      tolerancePose.getTranslation()
    );
  }

  protected Pose2d getNewTarget(Pose2d currTarget, Pose2d currentPose) {
    boolean isAtFinalApproach =
      Math.abs(Utility.findDistanceBetweenPoses(currentPose, m_finalGoal)) <=
      FINAL_APPROACH_DISTANCE.in(Meters);
    if (isAtFinalApproach && m_finalApproachCommand != null) {
      m_finalApproachCommand.schedule();
    }
    // if we can go to the final goal without hitting it, just go there
    if (isAtFinalApproach) {
      currTarget = m_finalGoal;
      if (
        m_currDriveToPose.getDriveConstraints().maxVelocity >
        DRIVE_FINAL_APPROACH_CONSTRAINTS.maxVelocity
      ) {
        m_currDriveToPose.setDriveConstraints(DRIVE_FINAL_APPROACH_CONSTRAINTS);
      }
      return m_finalGoal;
    } else if (
      m_currDriveToPose.getDriveConstraints().maxVelocity <
      DRIVE_DEFAULT_CONSTRAINTS.maxVelocity
    ) {
      m_currDriveToPose.setDriveConstraints(DRIVE_DEFAULT_CONSTRAINTS);
    }
    if (
      !isAtTarget(currTarget, currentPose, WAYPOINT_APPROACH_TOLERANCE_POSE)
    ) {
      return m_currentToldTarget;
    }

    Pose2d newTarget = interpolateTarget(currentPose, m_finalGoal);
    m_currentTarget = newTarget;
    m_currentToldTarget = newTarget.transformBy(
      new Transform2d(currentPose, newTarget)
    );
    return newTarget.transformBy(new Transform2d(currentPose, newTarget));
  }

  /**
   * Interpolates a new target based on the constant for interpolation distance.
   * @param currentPose The current pose
   * @param goal The final goal
   * @return The interpolated pose
   */
  protected Pose2d interpolateTarget(Pose2d currentPose, Pose2d goal) {
    Transform2d currPoseToGoalTransform = new Transform2d(
      new Pose2d(currentPose.getTranslation(), Rotation2d.kZero),
      new Pose2d(goal.getTranslation(), Rotation2d.kZero)
    );
    double dist = Math.abs(currPoseToGoalTransform.getTranslation().getNorm());
    Pose2d newTarget = currentPose.interpolate(
      goal,
      (1 / dist) * INTERPOLATION_DISTANCE.in(Meters)
    );
    return newTarget;
  }
}

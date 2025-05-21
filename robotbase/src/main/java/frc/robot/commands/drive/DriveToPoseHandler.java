package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.DRIVE_TO_POSE.*;

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
    m_currDriveToPose = new DriveToPose((Pose2d currPose) ->
      getNewTarget(m_currentTarget, currPose)
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
    Pose2d currPose,
    Pose2d tolerancePose
  ) {
    return Utility.isWithinTolerance(
      currPose.getTranslation(),
      targetPose.getTranslation(),
      tolerancePose.getTranslation()
    );
  }

  protected Pose2d getNewTarget(Pose2d currTarget, Pose2d currPose) {
    boolean isAtFinalApproach =
      Math.abs(Utility.findDistanceBetweenPoses(currPose, m_finalGoal)) <=
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
    if (!isAtTarget(currTarget, currPose, WAYPOINT_APPROACH_TOLERANCE_POSE)) {
      return m_currentToldTarget;
    }

    Pose2d newTarget = interpolateTarget(currPose, m_finalGoal);
    // if (
    //   m_routeAroundReef != RouteAroundReef.None &&
    //   CollisionDetection.willHitReef(
    //     currPose,
    //     newTarget,
    //     DEFAULT_INTERPOLATION_PERCENTAGES
    //   )
    // ) {
    //   newTarget = avoidReef(currPose, newTarget, m_routeAroundReef);
    // }
    m_currentTarget = newTarget;
    m_currentToldTarget = newTarget.transformBy(
      new Transform2d(currPose, newTarget).times(0.75)
    );
    return newTarget.transformBy(
      new Transform2d(currPose, newTarget).times(0.75)
    );
  }

  /**
   * Interpolates a new target based on the constant for interpolation distance.
   * @param currPose The current pose
   * @param goal The final goal
   * @return The interpolated pose
   */
  protected Pose2d interpolateTarget(Pose2d currPose, Pose2d goal) {
    Transform2d currPoseToGoalTransform = new Transform2d(
      new Pose2d(currPose.getTranslation(), Rotation2d.kZero),
      new Pose2d(goal.getTranslation(), Rotation2d.kZero)
    );
    double dist = Math.abs(currPoseToGoalTransform.getTranslation().getNorm());
    Pose2d newTarget = currPose.interpolate(
      goal,
      (1 / dist) * INTERPOLATION_DISTANCE.in(Meters)
    );
    return newTarget;
  }

  /**
   * Avoids any collision with the reef
   * @param currPose Robots current pose
   * @param newTarget The target that is predicted to hit the reef
   * @param routeAroundReef Chosen path around the reef
   * @return A pose that should be a target that follows the desired route around the reef, and going towards the goal.
   */
  protected Pose2d avoidReef(
    Pose2d currPose,
    Pose2d newTarget,
    RouteAroundReef routeAroundReef
  ) {
    if (routeAroundReef == RouteAroundReef.None) return newTarget;
    Pose2d target = currPose;
    Pose2d targetClockwise = new Pose2d(
      currPose
        .getTranslation()
        .rotateAround(
          REEF.CENTER.getTranslation(),
          ROTATE_AROUND_REEF_ROTATION
        ),
      currPose.getRotation()
    );
    Pose2d targetCounterClockwise = new Pose2d(
      currPose
        .getTranslation()
        .rotateAround(
          REEF.CENTER.getTranslation(),
          ROTATE_AROUND_REEF_ROTATION.unaryMinus()
        ),
      currPose.getRotation()
    );
    double clockwiseDistFromCenter = Math.abs(
      Utility.findDistanceBetweenPoses(m_finalGoal, targetClockwise)
    );
    double counterClockwiseDistFromCenter = Math.abs(
      Utility.findDistanceBetweenPoses(m_finalGoal, targetCounterClockwise)
    );
    switch (routeAroundReef) {
      case Clockwise:
        target = targetClockwise;
        break;
      case CounterClockwise:
        target = targetCounterClockwise;
        break;
      case Fastest:
        target = clockwiseDistFromCenter <= counterClockwiseDistFromCenter
          ? targetClockwise
          : targetCounterClockwise;
        break;
      case None:
        return newTarget;
      default:
        target = targetClockwise;
    }
    double distAwayFromReef = REEF.CENTER.getTranslation()
      .getDistance(target.getTranslation());
    if (distAwayFromReef < IDEAL_DISTANCE_FROM_REEF.in(Meters)) {
      Transform2d centerToTarTransform = new Transform2d(REEF.CENTER, target);
      centerToTarTransform.times(
        ((1 / distAwayFromReef) * IDEAL_DISTANCE_FROM_REEF.in(Meters))
      );
      target.transformBy(
        new Transform2d(centerToTarTransform.getTranslation(), Rotation2d.kZero)
      );
    }
    return target;
  }
}

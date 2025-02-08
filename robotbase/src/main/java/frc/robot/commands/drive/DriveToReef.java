package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.COLLISION_DETECTION.REEF_BOUNDARY;
import static frc.robot.Constants.COLLISION_DETECTION.REEF_SAT_POLY;
import static frc.robot.Constants.DRIVE_TO_POSE.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.util.SATCollisionDetector;
import frc.robot.util.Utility;
import java.util.function.Function;

public class DriveToReef extends Command {

  public enum DirectionOfTravel {
    Clockwise,
    CounterClockwise,
    Fastest,
  }

  private Pose2d m_currPose, m_currentTarget, m_finalGoal;

  private DriveToPose m_currDriveToPose;

  public DriveToReef() {}

  @Override
  public void initialize() {
    m_currentTarget = Robot.swerve.getAllianceRelativePose2d();
    m_finalGoal = Robot.buttonboard.getPoseFromGoal();
    m_currPose = Robot.swerve.getAllianceRelativePose2d();
    m_currDriveToPose = new DriveToPose(getTargetFunction()); // make a DriveToPose that we have control of
    m_currDriveToPose.schedule();
  }

  @Override
  public void execute() {
    m_finalGoal = Robot.buttonboard.getPoseFromGoal();
    m_currPose = Robot.swerve.getAllianceRelativePose2d();
  }

  @Override
  public boolean isFinished() {
    return isAtTarget(m_finalGoal, m_currPose);
  }

  @Override
  public void end(boolean isInteruptted) {
    m_currDriveToPose.cancel();
    Robot.swerve.stopMotors();
  }

  private boolean isAtTarget(Pose2d targetPose, Pose2d currPose) {
    if (
      !Utility.isWithinTolerance(
        targetPose.getX(),
        currPose.getX(),
        X_TOLERANCE.in(Meters)
      )
    ) {
      return false;
    }
    if (
      !Utility.isWithinTolerance(
        targetPose.getY(),
        currPose.getY(),
        Y_TOLERANCE.in(Meters)
      )
    ) {
      return false;
    }
    return true;
  }

  private Function<Pose2d, Pose2d> getTargetFunction() {
    return new Function<Pose2d, Pose2d>() {
      @Override
      public Pose2d apply(Pose2d currPose) {
        return findNewTarget(m_currentTarget, currPose);
      }
    };
  }

  public boolean willHitReef(
    Pose2d currPose,
    Pose2d targetPose,
    double... interpolationPercentages
  ) {
    Transform2d currToTargetTransform = new Transform2d(currPose, targetPose);
    for (double percentage : interpolationPercentages) {
      Transform2d transformToUse = currToTargetTransform.times(percentage);
      Pose2d interpolatedPose = currPose.transformBy(transformToUse);
      // if true, collision with reef is likely, and avoidance should begin.
      if (
        SATCollisionDetector.hasCollided(
          SATCollisionDetector.makePolyFromRobotPose(interpolatedPose),
          REEF_SAT_POLY,
          REEF_BOUNDARY.in(Meters)
        )
      ) {
        return true;
      }
    }
    return false;
  }

  private Pose2d findNewTarget(Pose2d currTarget, Pose2d currPose) {
    // if we can go to the final goal without hitting it, just go there
    if (
      Math.abs(Utility.findDistanceBetweenPoses(currPose, m_finalGoal)) <=
        FINAL_APPROACH_DISTANCE.in(Meters) ||
      !willHitReef(currPose, m_finalGoal, DEFAULT_INTERPOLATION_PERCENTAGES)
    ) {
      m_currentTarget = m_finalGoal;
      return m_finalGoal;
    }

    if (!isAtTarget(currTarget, currPose)) {
      // make it go faster by lying to it
      return currTarget.transformBy(new Transform2d(currPose, currTarget));
    }

    Pose2d newTarget = interpolateTarget(currPose, m_finalGoal);
    if (willHitReef(currPose, newTarget, DEFAULT_INTERPOLATION_PERCENTAGES)) {
      newTarget = rotateAway(currPose, DirectionOfTravel.Fastest);
    }
    m_currentTarget = newTarget;
    // make it go faster through deceit and deception
    return newTarget.transformBy(new Transform2d(currPose, newTarget));
  }

  /**
   * Interpolates a new target based on the constant for interpolation distance.
   * @param currPose The current pose
   * @param goal The final goal
   * @return The interpolated pose
   */
  private Pose2d interpolateTarget(Pose2d currPose, Pose2d goal) {
    double dist = Utility.findDistanceBetweenPoses(currPose, goal);
    return currPose.interpolate(
      goal,
      (1 / dist) * INTERPOLATION_DISTANCE.in(Meters)
    );
  }

  private Pose2d rotateAway(
    Pose2d currPose,
    DirectionOfTravel routeAroundReef
  ) {
    Pose2d target;
    Pose2d targetClockwise = new Pose2d(
      currPose
        .getTranslation()
        .rotateAround(
          REEF.CENTER.getTranslation(),
          ROTATE_AROUND_REEF_ROTATIONS
        ),
      currPose.getRotation()
    );
    Pose2d targetCounterClockwise = new Pose2d(
      currPose
        .getTranslation()
        .rotateAround(
          REEF.CENTER.getTranslation(),
          ROTATE_AROUND_REEF_ROTATIONS.unaryMinus()
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
      default:
        target = targetClockwise;
    }
    double distAwayFromReef = Utility.findDistanceBetweenPoses(
      REEF.CENTER,
      target
    );
    if (distAwayFromReef < IDEAL_DISTANCE_FROM_REEF.in(Meters)) {
      Transform2d centerToTarTransform = new Transform2d(REEF.CENTER, target);
      centerToTarTransform.times(
        ((1 / distAwayFromReef) * IDEAL_DISTANCE_FROM_REEF.in(Meters))
      );
      target.transformBy(centerToTarTransform);
    }
    return target;
  }
}

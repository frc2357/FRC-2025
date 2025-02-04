package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.DRIVE_TO_POSE.COLLISION_AVOIDANCE.DEFAULT_INTERPOLATION_PERCENTAGES;
import static frc.robot.Constants.DRIVE_TO_POSE.COLLISION_AVOIDANCE.IDEAL_DISTANCE_FROM_REEF;
import static frc.robot.Constants.DRIVE_TO_POSE.FINAL_APPROACH_DISTANCE;
import static frc.robot.Constants.DRIVE_TO_POSE.INTERPOLATION_PERCENT;
import static frc.robot.Constants.DRIVE_TO_POSE.ROTATION_TOLERANCE;
import static frc.robot.Constants.DRIVE_TO_POSE.X_TOLERANCE;
import static frc.robot.Constants.DRIVE_TO_POSE.Y_TOLERANCE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DRIVE_TO_POSE.COLLISION_AVOIDANCE;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.util.Utility;
import java.util.function.Function;

public class DriveToReef extends Command {

  private enum DirectionOfTravel {
    X,
    Y,
  }

  private Pose2d m_currPose;

  private Pose2d m_currentTarget;
  private Pose2d m_lastTarget;

  private Pose2d m_finalGoal;

  private DriveToPose m_currDriveToPose;

  public DriveToReef() {}

  @Override
  public void initialize() {
    m_lastTarget = Robot.swerve.getAllianceRelativePose2d();
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
    if (
      !Utility.isWithinTolerance(
        targetPose.getRotation().getDegrees(),
        currPose.getRotation().getDegrees(),
        ROTATION_TOLERANCE.in(Degrees)
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
        return findNewTarget(m_currentTarget, m_lastTarget, currPose);
      }
    };
  }

  private Pose2d getPoseDelta(Pose2d origin, Pose2d delta) {
    return delta.relativeTo(origin);
  }

  private boolean willHitReef(
    Pose2d currPose,
    Pose2d targetPose,
    double... interpolationPercentages
  ) {
    for (double percentage : interpolationPercentages) {
      Pose2d interpolatedPose = currPose.interpolate(targetPose, percentage);
      // if true, collision with reef is likely, and avoidance should begin.

      if (
        Math.abs(
          Utility.findDistanceBetweenPoses(interpolatedPose, REEF.CENTER)
        ) <=
        COLLISION_AVOIDANCE.REEF_BOUNDARY.in(Meters)
      ) {
        return true;
      }
    }
    return false;
  }

  private Pose2d findNewTarget(
    Pose2d currTarget,
    Pose2d lastTarget,
    Pose2d currPose
  ) {
    Pose2d currPoseToFinalGoalDelta = getPoseDelta(currPose, m_finalGoal);
    // if were close to the final goal, just make the target the goal and send it
    if (
      currPoseToFinalGoalDelta.getX() < FINAL_APPROACH_DISTANCE.in(Meters) &&
      currPoseToFinalGoalDelta.getY() < FINAL_APPROACH_DISTANCE.in(Meters)
    ) {
      m_currentTarget = m_finalGoal;
      return m_finalGoal; // cant use collision avoidance, would make sure we dont get that close
    }

    Pose2d newTarget = currPose.interpolate(m_finalGoal, INTERPOLATION_PERCENT);
    if (willHitReef(currPose, newTarget, DEFAULT_INTERPOLATION_PERCENTAGES)) {
      newTarget = pinPoseToReef(
        newTarget,
        findMainDirectionOfTravel(currPose, m_finalGoal)
      );
    }
    m_lastTarget = m_currentTarget;
    m_currentTarget = newTarget;
    return newTarget;
  }

  private DirectionOfTravel findMainDirectionOfTravel(
    Pose2d currPose,
    Pose2d finalGoal
  ) {
    Pose2d currPoseToGoalDelta = getPoseDelta(currPose, finalGoal);
    return currPoseToGoalDelta.getX() >= currPoseToGoalDelta.getY()
      ? DirectionOfTravel.X
      : DirectionOfTravel.Y;
  }

  private Pose2d pinPoseToReef(
    Pose2d poseToPin,
    DirectionOfTravel pinnedDirection
  ) { // c^2 - a^2 = b^2 | c^2 - b^2 = a^2 | a = x, b = y, change whatever is NOT the pinned direction
    Pose2d poseToCenterDelta = getPoseDelta(REEF.CENTER, poseToPin);
    double a = poseToCenterDelta.getX();
    double b = poseToCenterDelta.getY();

    // pinned pose is relative to the center of the reef
    Pose2d pinnedPose = Pose2d.kZero;

    Pose2d upRightPinned = Pose2d.kZero;
    Pose2d downLeftpinned = Pose2d.kZero;
    double upRightDist = 0;
    double downLeftDist = 0;
    switch (pinnedDirection) {
      case X:
        upRightPinned = new Pose2d(
          a,
          Math.sqrt(
            Math.pow(IDEAL_DISTANCE_FROM_REEF.in(Meters), 2) - Math.pow(a, 2)
          ),
          Rotation2d.kZero
        );
        downLeftpinned = new Pose2d(
          a,
          Math.sqrt(
            Math.pow(IDEAL_DISTANCE_FROM_REEF.in(Meters), 2) - Math.pow(a, 2)
          ) *
          -1,
          Rotation2d.kZero
        );
        upRightDist = Math.abs(
          Utility.findDistanceBetweenPoses(pinnedPose, upRightPinned)
        );
        downLeftDist = Math.abs(
          Utility.findDistanceBetweenPoses(pinnedPose, downLeftpinned)
        );
        pinnedPose = upRightDist < downLeftDist
          ? upRightPinned
          : downLeftpinned;
        break;
      case Y:
        upRightPinned = new Pose2d(
          Math.sqrt(
            Math.pow(IDEAL_DISTANCE_FROM_REEF.in(Meters), 2) - Math.pow(b, 2)
          ),
          b,
          Rotation2d.kZero
        );
        downLeftpinned = new Pose2d(
          Math.sqrt(
            Math.pow(IDEAL_DISTANCE_FROM_REEF.in(Meters), 2) - Math.pow(b, 2)
          ) *
          -1,
          b,
          Rotation2d.kZero
        );
        upRightDist = Math.abs(
          Utility.findDistanceBetweenPoses(pinnedPose, upRightPinned)
        );
        downLeftDist = Math.abs(
          Utility.findDistanceBetweenPoses(pinnedPose, downLeftpinned)
        );
        pinnedPose = upRightDist < downLeftDist
          ? upRightPinned
          : downLeftpinned;
        break;
    }
    Pose2d finalPose = pinnedPose.plus(Utility.poseToTransform(REEF.CENTER));
    // If the poseToPin is close to being pinned, the changed coord will be NaN.
    // This is the easiest way to make sure we dont give a NaN
    if (Double.isNaN(finalPose.getX())) {
      finalPose = new Pose2d(
        poseToPin.getX(),
        finalPose.getY(),
        poseToPin.getRotation()
      );
    }
    if (Double.isNaN(finalPose.getY())) {
      finalPose = new Pose2d(
        finalPose.getX(),
        poseToPin.getY(),
        poseToPin.getRotation()
      );
    }
    return finalPose;
  }
}

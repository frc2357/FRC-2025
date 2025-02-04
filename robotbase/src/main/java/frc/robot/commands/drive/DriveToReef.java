package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.DRIVE_TO_POSE.COLLISION_AVOIDANCE.*;
import static frc.robot.Constants.DRIVE_TO_POSE.FINAL_APPROACH_DISTANCE;
import static frc.robot.Constants.DRIVE_TO_POSE.INTERPLOATION_PERCENT;
import static frc.robot.Constants.DRIVE_TO_POSE.ROTATION_TOLERANCE;
import static frc.robot.Constants.DRIVE_TO_POSE.X_TOLERANCE;
import static frc.robot.Constants.DRIVE_TO_POSE.Y_TOLERANCE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DSControlWord;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DRIVE_TO_POSE.COLLISION_AVOIDANCE;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.util.Utility;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;

public class DriveToReef extends Command {

  private enum DirectionOfTravel {
    X(true),
    Y(false);

    private final boolean dirOfTravel;

    DirectionOfTravel(boolean dirOfTravel) {
      this.dirOfTravel = dirOfTravel;
    }
  }

  private Pose2d m_currPose;

  private Pose2d m_currentTarget;
  private Pose2d m_lastTarget;

  private Pose2d m_finalGoal;

  private DriveToPose m_currDriveToPose;

  private Twist2d m_collisionAvoidanceTwist = new Twist2d(
    TWIST_X_METERS_DEFAULT,
    TWIST_Y_METERS_DEFAULT,
    TWIST_ROTO_RADIANS_DEFAULT
  );

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
      double dist = Utility.findDistanceBetweenPoses(
        interpolatedPose,
        REEF.CENTER
      );

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

  private boolean isFinalGoal(Pose2d targetPose) {
    return targetPose.equals(m_finalGoal);
  }

  /**
   * Takes a pose, and returns a pose that should NOT hit the reef. most likely.
   * @param currTarget The robots current target
   * @param lastTarget The robots last target
   * @param currPose The robots current pose
   * @return A pose to use a target that shouldnt hit the reef.
   */
  private Pose2d collisionAvoidanceTarget(
    Pose2d currTarget,
    Pose2d lastTarget,
    Pose2d currPose
  ) {
    Pose2d safeTarget = currTarget;
    Pose2d lastPoseToCurrTarDelta = getPoseDelta(currPose, currTarget);

    // find which direction we want to go (pos or neg in x/y axes)
    // 1 means we want to go UP in value, -1 means we want to go DOWN in value.
    double xDirectionOfTravelMult = lastPoseToCurrTarDelta.getX() >= 0 ? 1 : -1;
    double yDirectionOfTravelMult = lastPoseToCurrTarDelta.getY() >= 0 ? 1 : -1;
    // figure out whether or not were going to hit the reef
    if (
      willHitReef(currPose, currTarget, 0.9, 0.7, 0.5) &&
      !isFinalGoal(currTarget)
    ) {
      m_collisionAvoidanceTwist.dx =
        COLLISION_AVOIDANCE.TWIST_X_METERS_DEFAULT * xDirectionOfTravelMult;
      m_collisionAvoidanceTwist.dy =
        COLLISION_AVOIDANCE.TWIST_Y_METERS_DEFAULT * yDirectionOfTravelMult;
      m_collisionAvoidanceTwist.dtheta =
        COLLISION_AVOIDANCE.TWIST_ROTO_RADIANS_DEFAULT;
      int attemptsTaken = 0;
      for (
        attemptsTaken = 0;
        attemptsTaken < COLLISION_AVOIDANCE.ATTEMPTS;
        attemptsTaken++
      ) {
        safeTarget.exp(m_collisionAvoidanceTwist);
        if (!willHitReef(currPose, safeTarget, INTERPLOATION_PERCENT)) {
          break;
        }
        m_collisionAvoidanceTwist.dx += m_collisionAvoidanceTwist.dx / 2;
        m_collisionAvoidanceTwist.dy += m_collisionAvoidanceTwist.dy / 2;
      }
      // make sure that the safeTarget is not past the final goal
      if (isBeyondTarget(safeTarget, lastTarget, m_finalGoal)) {
        Pose2d safeTarToFinalGoalDelta = getPoseDelta(safeTarget, m_finalGoal);
        if (safeTarToFinalGoalDelta.getX() < 0) {
          safeTarget = new Pose2d(
            m_finalGoal.getX(),
            safeTarget.getY(),
            safeTarget.getRotation()
          );
        }
        if (safeTarToFinalGoalDelta.getY() < 0) {
          safeTarget = new Pose2d(
            safeTarget.getX(),
            m_finalGoal.getY(),
            safeTarget.getRotation()
          );
        }
      }
    }
    return safeTarget;
  }

  /**
   * Returns whether or not your current pose is beyond your current target
   * @param currTarget Used to determine where you want to go
   * @param lastTarget Used to determine which direction your going
   * @param currPose Your pose, so we know where you are and can bound correctly
   * @return Whether or not your currPose is beyond your currTarget in the x or y axis
   */
  private boolean isBeyondTarget(
    Pose2d currTarget,
    Pose2d lastTarget,
    Pose2d currPose
  ) {
    Pose2d lastTarToCurrTarDelta = getPoseDelta(lastTarget, currTarget);
    Pose2d currPoseToCurrTarDelta = getPoseDelta(currPose, currTarget);

    // find which direction we want to go (pos or neg in x/y axes)
    // 1 means we want to go UP in value, -1 means we want to go DOWN in value.
    double xDirectionOfTravelMult = lastTarToCurrTarDelta.getX() >= 0 ? 1 : -1;
    double yDirectionOfTravelMult = lastTarToCurrTarDelta.getY() >= 0 ? 1 : -1;
    if (
      currPoseToCurrTarDelta.getX() !=
      Math.copySign(currPoseToCurrTarDelta.getX(), xDirectionOfTravelMult)
    ) {
      return true;
    }
    if (
      currPoseToCurrTarDelta.getY() !=
      Math.copySign(currPoseToCurrTarDelta.getY(), yDirectionOfTravelMult)
    ) {
      return true;
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

    Pose2d newTarget = currPose.interpolate(m_finalGoal, INTERPLOATION_PERCENT);
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

  private void setFinalGoal(Pose2d goal) {
    m_finalGoal = goal;
  }
}

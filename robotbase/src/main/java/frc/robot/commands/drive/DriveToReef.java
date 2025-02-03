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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DRIVE_TO_POSE.COLLISION_AVOIDANCE;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.util.Utility;
import java.util.function.Function;

public class DriveToReef extends Command {

  private enum DirectionOfTravel {
    X(true),
    Y(false);

    public final boolean dirOfTravel;

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

    // if not beyond current target, keep using it
    if (!isBeyondTarget(currTarget, lastTarget, currPose)) {
      return currTarget;
    }
    Pose2d newTarget = currPose.interpolate(m_finalGoal, INTERPLOATION_PERCENT);
    newTarget = pinPoseToReef(
      newTarget,
      findMainDirectionOfTravel(currPose, m_finalGoal)
    );
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
  ) { // c^2 - a^2 = b^2 | c^2 - b^2 = a^2 | a = x, b = y, change whatever is NOT the main direction of travel
    Pose2d poseToCenterDelta = getPoseDelta(poseToPin, REEF.CENTER);
    double a = poseToCenterDelta.getX();
    double b = poseToCenterDelta.getY();

    Transform2d pinnedTransform = Transform2d.kZero;

    switch (pinnedDirection) {
      case X:
        pinnedTransform = new Transform2d(
          0,
          Math.sqrt(
            Math.pow(IDEAL_DISTANCE_FROM_REEF.in(Meters), 2) - Math.pow(a, 2)
          ),
          Rotation2d.kZero
        );
        break;
      case Y:
        pinnedTransform = new Transform2d(
          0,
          Math.sqrt(
            Math.pow(IDEAL_DISTANCE_FROM_REEF.in(Meters), 2) - Math.pow(b, 2)
          ),
          Rotation2d.kZero
        );
        break;
    }

    return poseToPin.transformBy(pinnedTransform);
  }
}

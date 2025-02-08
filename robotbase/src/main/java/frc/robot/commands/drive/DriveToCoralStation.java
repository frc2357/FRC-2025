package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.DRIVE_TO_POSE.*;
import static frc.robot.Constants.FIELD.CORAL_STATION.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.commands.drive.DriveToReef.RouteAroundReef;
import frc.robot.util.CollisionDetection;
import frc.robot.util.Utility;
import java.util.function.Function;

public class DriveToCoralStation extends Command {

  public enum StationToGoTo {
    Upper,
    Lower,
    Fastest,
  }

  private Pose2d m_currPose, m_currentTarget, m_finalGoal;

  private DriveToPose m_currDriveToPose;

  private StationToGoTo m_desiredStation;

  private RouteAroundReef m_routeAroundReef;

  public DriveToCoralStation(
    StationToGoTo desiredStation,
    RouteAroundReef routeAroundReef
  ) {
    m_desiredStation = desiredStation != null
      ? desiredStation
      : StationToGoTo.Fastest;
    m_routeAroundReef = routeAroundReef;
  }

  @Override
  public void initialize() {
    m_currentTarget = Robot.swerve.getAllianceRelativePose2d();
    m_currPose = Robot.swerve.getAllianceRelativePose2d();
    m_currDriveToPose = new DriveToPose(getTargetFunction()); // make a DriveToPose that we have control of
    m_currDriveToPose.schedule();
  }

  @Override
  public void execute() {
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

  private Pose2d findNewTarget(Pose2d currTarget, Pose2d currPose) {
    // if we can go to the final goal without hitting the reef, just go there
    m_finalGoal = getDesiredTarget(currPose);
    if (
      Math.abs(Utility.findDistanceBetweenPoses(currPose, m_finalGoal)) <=
        FINAL_APPROACH_DISTANCE.in(Meters) ||
      !CollisionDetection.willHitReef(
        currPose,
        m_finalGoal,
        DEFAULT_INTERPOLATION_PERCENTAGES
      )
    ) {
      m_currentTarget = m_finalGoal;
      return m_finalGoal;
    }

    if (!isAtTarget(currTarget, currPose)) {
      // make it go faster by lying to it
      return currTarget.transformBy(new Transform2d(currPose, currTarget));
    }

    Pose2d newTarget = interpolateTarget(currPose, m_finalGoal);
    if (
      CollisionDetection.willHitReef(
        currPose,
        newTarget,
        DEFAULT_INTERPOLATION_PERCENTAGES
      )
    ) {
      newTarget = rotateAway(currPose, m_routeAroundReef);
    }
    m_currentTarget = newTarget;
    // make it go faster through deceit and deception
    return newTarget.transformBy(new Transform2d(currPose, newTarget));
  }

  private Pose2d getDesiredTarget(Pose2d currPose) {
    switch (m_desiredStation) {
      case Lower:
        return LOWER_STATION_DESIRED_SLOT;
      case Upper:
        return UPPER_STATION_DESIRED_SLOT;
      case Fastest:
      default:
        double upperStationDist = Utility.findDistanceBetweenPoses(
          currPose,
          UPPER_STATION_DESIRED_SLOT
        );
        double lowerStationDist = Utility.findDistanceBetweenPoses(
          currPose,
          LOWER_STATION_DESIRED_SLOT
        );
        return upperStationDist <= lowerStationDist
          ? UPPER_STATION_DESIRED_SLOT
          : LOWER_STATION_DESIRED_SLOT;
    }
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

  private Pose2d rotateAway(Pose2d currPose, RouteAroundReef routeAroundReef) {
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

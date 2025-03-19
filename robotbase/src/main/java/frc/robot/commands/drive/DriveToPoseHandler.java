package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.DRIVE_TO_POSE.*;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.util.CollisionDetection;
import frc.robot.util.Utility;
import java.util.function.Function;

public class DriveToPoseHandler extends Command {

  public enum RouteAroundReef {
    Clockwise,
    CounterClockwise,
    Fastest,
    None,
  }

  protected Pose2d m_currPose, m_currentTarget, m_finalGoal;

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
    m_currentTarget = Robot.swerve.getAllianceRelativePose2d();
    // m_finalGoal = Robot.buttonboard.getPoseFromGoal();
    m_currPose = Robot.swerve.getAllianceRelativePose2d();
    m_currDriveToPose = new DriveToPose(getTargetFunction()); // make a DriveToPose that we have control of
    m_currDriveToPose.schedule();
  }

  @Override
  public void execute() {
    // m_finalGoal = Robot.buttonboard.getPoseFromGoal();
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

  protected boolean isAtTarget(Pose2d targetPose, Pose2d currPose) {
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

  protected Function<Pose2d, Pose2d> getTargetFunction() {
    return new Function<Pose2d, Pose2d>() {
      @Override
      public Pose2d apply(Pose2d currPose) {
        return getNewTarget(m_currentTarget, currPose);
      }
    };
  }

  protected Pose2d getNewTarget(Pose2d currTarget, Pose2d currPose) {
    // boolean isAtFinalApproach =
    //   Math.abs(
    //     new Transform2d(currPose, m_finalGoal).getTranslation().getNorm()
    //   ) <=
    //   FINAL_APPROACH_DISTANCE.in(Meters);
    // if (isAtFinalApproach && m_finalApproachCommand != null) {
    //   m_finalApproachCommand.schedule();
    // }
    // if we can go to the final goal without hitting it, just go there
    // if (isAtFinalApproach) {
    //   m_currentTarget = m_finalGoal;
    //   return m_finalGoal;
    // }

    if (!isAtTarget(currTarget, currPose)) {
      // make it go faster by lying to it
      return currTarget/*.transformBy(new Transform2d(currPose, currTarget))*/;
    }

    Pose2d newTarget = interpolateTarget(currPose, m_finalGoal);
    if (
      m_routeAroundReef != RouteAroundReef.None &&
      CollisionDetection.willHitReef(
        currPose,
        newTarget,
        DEFAULT_INTERPOLATION_PERCENTAGES
      )
    ) {
      newTarget = avoidReef(currPose, m_routeAroundReef);
    }
    m_currentTarget = newTarget;
    // make it go faster through deceit and deception
    return newTarget/*.transformBy(new Transform2d(currPose, newTarget)) */;
  }

  /**
   * Interpolates a new target based on the constant for interpolation distance.
   * @param currPose The current pose
   * @param goal The final goal
   * @return The interpolated pose
   */
  protected Pose2d interpolateTarget(Pose2d currPose, Pose2d goal) {
    double dist = Utility.findDistanceBetweenPoses(currPose, goal);
    return currPose.interpolate(
      goal,
      0.4
      // (1 / dist) * INTERPOLATION_DISTANCE.in(Meters)
    );
  }

  /**
   * Avoids any collision with the reef
   * @param currPose Robots current pose
   * @param routeAroundReef Chosen path around the reef
   * @return A pose that should be a target that follows the desired route around the reef, and going towards the goal.
   */
  protected Pose2d avoidReef(Pose2d currPose, RouteAroundReef routeAroundReef) {
    if (routeAroundReef == RouteAroundReef.None) return currPose;
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

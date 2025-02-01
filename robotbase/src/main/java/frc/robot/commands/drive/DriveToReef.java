package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.DRIVE_TO_POSE.FINAL_APPROACH_DISTANCE;
import static frc.robot.Constants.DRIVE_TO_POSE.INTERPLOATION_PERCENT;
import static frc.robot.Constants.DRIVE_TO_POSE.ROTATION_TOLERANCE;
import static frc.robot.Constants.DRIVE_TO_POSE.X_TOLERANCE;
import static frc.robot.Constants.DRIVE_TO_POSE.Y_TOLERANCE;
import static frc.robot.Constants.FIELD.REEF.BRANCH_A;
import static frc.robot.Constants.FIELD.REEF.BRANCH_B;
import static frc.robot.Constants.FIELD.REEF.BRANCH_C;
import static frc.robot.Constants.FIELD.REEF.BRANCH_D;
import static frc.robot.Constants.FIELD.REEF.BRANCH_E;
import static frc.robot.Constants.FIELD.REEF.BRANCH_F;
import static frc.robot.Constants.FIELD.REEF.BRANCH_G;
import static frc.robot.Constants.FIELD.REEF.BRANCH_H;
import static frc.robot.Constants.FIELD.REEF.BRANCH_I;
import static frc.robot.Constants.FIELD.REEF.BRANCH_J;
import static frc.robot.Constants.FIELD.REEF.BRANCH_K;
import static frc.robot.Constants.FIELD.REEF.BRANCH_L;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DRIVE_TO_POSE.COLLISION_AVOIDANCE;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.controls.controllers.ButtonboardController.ReefSide;
import frc.robot.controls.controllers.ButtonboardController.ScoringDirection;
import frc.robot.util.Utility;
import java.util.function.Function;

public class DriveToReef extends Command {

  private Pose2d m_currPose;

  private Pose2d m_currentTarget;
  private Pose2d m_lastTarget;

  private Pose2d m_finalGoal;

  private DriveToPose m_currDriveToPose;

  private Twist2d m_collisionAvoidanceTwist = new Twist2d();

  private boolean m_isDriverControlling;

  public DriveToReef() {}

  @Override
  public void initialize() {
    m_lastTarget = Robot.swerve.getAllianceRelativePose2d();
    m_finalGoal = getGoalFromButtonboard();
    // if final goal is null, then somethings up, and we dont have any clue where to go from here.
    if (m_finalGoal == null) {
      m_currDriveToPose.cancel();
      this.cancel(); // so we wont go anywhere from here, and just cancel instead.
      return;
    }
    m_currPose = Robot.swerve.getAllianceRelativePose2d();
    m_currDriveToPose = new DriveToPose(getTargetFunction()); // make a DriveToPose that we have control of
    m_currDriveToPose.schedule();
  }

  @Override
  public void execute() {
    checkDriverInputs();
    m_finalGoal = getGoalFromButtonboard();
    if (m_finalGoal == null) {
      if (m_finalGoal == null) {
        m_currDriveToPose.cancel();
        this.cancel(); // nowhere to go, so no purpose.
        return;
      }
    }
    m_currPose = Robot.swerve.getAllianceRelativePose2d();
  }

  @Override
  public boolean isFinished() {
    if (!m_currDriveToPose.isScheduled()) {
      if (!isAtGoal()) {
        m_currDriveToPose = new DriveToPose(getTargetFunction());
        return false;
      }
    }
    return isAtGoal();
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

  private boolean isAtGoal() {
    return isAtTarget(m_finalGoal, m_currPose);
  }

  private Pose2d getGoalFromButtonboard() {
    ReefSide goal = Robot.buttonboard.getSelectedReefSide();
    ScoringDirection scoringDirection =
      Robot.buttonboard.getSelectedScoringDirection();

    switch (scoringDirection) {
      case Left:
        switch (goal) {
          case A:
            return BRANCH_A;
          case B:
            return BRANCH_C;
          case C:
            return BRANCH_E;
          case D:
            return BRANCH_G;
          case E:
            return BRANCH_I;
          case F:
            return BRANCH_K;
          default:
            return null; // if this happens, somethings wrong.
        }
      case Right:
        switch (goal) {
          case A:
            return BRANCH_B;
          case B:
            return BRANCH_D;
          case C:
            return BRANCH_F;
          case D:
            return BRANCH_H;
          case E:
            return BRANCH_J;
          case F:
            return BRANCH_L;
          default:
            return null; // if this happens, somethings wrong.
        }
      default:
        return null; // if this happens, somethings wrong.
    }
  }

  private Function<Pose2d, Pose2d> getTargetFunction() {
    return new Function<Pose2d, Pose2d>() {
      @Override
      public Pose2d apply(Pose2d currPose) {
        return findNewTarget(m_currentTarget, m_lastTarget, currPose);
      }
    };
  }

  private Pose2d getPoseDelta(Pose2d currPose, Pose2d targetPose) {
    return targetPose.relativeTo(currPose);
  }

  private boolean arePoseTranslationsEqual(Pose2d pose1, Pose2d pose2) {
    return pose1.getX() == pose2.getX() && pose1.getY() == pose2.getY();
  }

  private boolean isFinalGoal(Pose2d targetPose) {
    return (
      arePoseTranslationsEqual(targetPose, m_finalGoal) &&
      targetPose.getRotation().getDegrees() ==
      m_finalGoal.getRotation().getDegrees()
    );
  }

  private boolean willHitReef(Pose2d currPose, Pose2d targetPose) {
    return willHitReef(currPose, targetPose, INTERPLOATION_PERCENT);
  }

  private boolean willHitReef(
    Pose2d currPose,
    Pose2d targetPose,
    double... interpolationPercentages
  ) {
    for (double percentage : interpolationPercentages) {
      if (willHitReef(currPose, targetPose, percentage)) {
        return true;
      }
    }
    return false;
  }

  private boolean willHitReef(
    Pose2d currPose,
    Pose2d targetPose,
    double interpolationPercentage
  ) {
    var interpolatedPose = currPose.interpolate(
      targetPose,
      interpolationPercentage
    );
    // if true, collision with reef is likely, and avoidance should begin.
    return (
      Math.abs(
        Utility.findDistanceBetweenPoses(interpolatedPose, REEF.CENTER)
      ) <=
      COLLISION_AVOIDANCE.REEF_BOUNDARY.in(Meters)
    );
  }

  private void checkDriverInputs() {
    if (
      Robot.driverControls.getX() != 0 ||
      Robot.driverControls.getY() != 0 ||
      Robot.driverControls.getRotation() != 0
    ) {
      m_isDriverControlling = true;
    }
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
    Pose2d validTarget = currTarget;
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

      Pose2d safeTarget = validTarget;
      int attemptsTaken = 0;
      for (
        attemptsTaken = 0;
        attemptsTaken < COLLISION_AVOIDANCE.ATTEMPTS;
        attemptsTaken++
      ) {
        safeTarget.exp(m_collisionAvoidanceTwist);
        if (!willHitReef(currPose, safeTarget)) {
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
      validTarget = safeTarget;
    }
    return validTarget;
  }

  private Pose2d findNewTarget(
    Pose2d currTarget,
    Pose2d lastTarget,
    Pose2d currPose
  ) {
    // if not beyond current target, keep using it
    if (!isBeyondTarget(currTarget, lastTarget, currPose)) {
      return collisionAvoidanceTarget(currTarget, lastTarget, currPose);
    }

    Pose2d currPoseToFinalGoalDelta = getPoseDelta(currPose, m_finalGoal);
    // if were close to the final goal, just make the target the goal and send it
    if (
      currPoseToFinalGoalDelta.getX() < FINAL_APPROACH_DISTANCE.in(Meters) &&
      currPoseToFinalGoalDelta.getY() < FINAL_APPROACH_DISTANCE.in(Meters)
    ) {
      m_currentTarget = m_finalGoal;
      return m_finalGoal; // cant use collision avoidance, would make sure we dont get that close
    }
    // interpolate a new target, and let collision avoidance make sure we dont slam into the reef
    Pose2d newTarget = currPose.interpolate(m_finalGoal, INTERPLOATION_PERCENT);
    m_lastTarget = m_currentTarget;
    m_currentTarget = newTarget;
    return collisionAvoidanceTarget(currTarget, lastTarget, currPose);
  }
}

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.DRIVE_TO_POSE.DEFAULT_INTERPOLATION_PERCENTAGES;
import static frc.robot.Constants.DRIVE_TO_POSE.FINAL_APPROACH_DISTANCE;
import static frc.robot.Constants.DRIVE_TO_POSE.IDEAL_DISTANCE_FROM_REEF;
import static frc.robot.Constants.DRIVE_TO_POSE.INTERPOLATION_DISTANCE;
import static frc.robot.Constants.DRIVE_TO_POSE.REEF_BOUNDARY;
import static frc.robot.Constants.DRIVE_TO_POSE.X_TOLERANCE;
import static frc.robot.Constants.DRIVE_TO_POSE.Y_TOLERANCE;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;
import frc.robot.util.Utility;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

public class DriveToReef extends Command {

  public enum DirectionOfTravel {
    X,
    Y,
    Left,
    Right,
  }

  public Pose2d m_currPose;

  public Pose2d m_currentTarget;
  public Pose2d m_lastTarget;

  public DirectionOfTravel m_lastDoT = DirectionOfTravel.X;

  public Pose2d m_finalGoal;

  public DriveToPose m_currDriveToPose;

  public DriveToReef() {}

  @Override
  public void initialize() {
    m_lastTarget = Robot.swerve.getAllianceRelativePose2d();
    m_currentTarget = Robot.swerve.getAllianceRelativePose2d();
    m_finalGoal = Robot.buttonboard.getPoseFromGoal();
    m_currPose = Robot.swerve.getAllianceRelativePose2d();
    m_currDriveToPose = new DriveToPose(
      getTargetFunction(),
      getIsFinishedSupplier()
    ); // make a DriveToPose that we have control of
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

  public boolean isAtTarget(Pose2d targetPose, Pose2d currPose) {
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
    // if (
    //   !Utility.isWithinTolerance(
    //     targetPose.getRotation().getDegrees(),
    //     currPose.getRotation().getDegrees(),
    //     ROTATION_TOLERANCE.in(Degrees)
    //   )
    // ) {
    //   return false;
    // }
    return true;
  }

  public Function<Pose2d, Pose2d> getTargetFunction() {
    return new Function<Pose2d, Pose2d>() {
      @Override
      public Pose2d apply(Pose2d currPose) {
        return findNewTarget(m_currentTarget, m_lastTarget, currPose);
      }
    };
  }

  public BooleanSupplier getIsFinishedSupplier() {
    return new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return isAtTarget(m_finalGoal, m_currPose);
      }
    };
  }

  public Pose2d getPoseDelta(Pose2d origin, Pose2d delta) {
    return delta.relativeTo(origin);
  }

  public boolean willHitReef(
    Pose2d currPose,
    Pose2d targetPose,
    double... interpolationPercentages
  ) {
    for (double percentage : interpolationPercentages) {
      Pose2d interpolatedPose = currPose.interpolate(targetPose, percentage);
      // if true, collision with reef is likely, and avoidance should begin.
      if (
        Math.abs(
          Utility.findDistanceBetweenPoses(REEF.CENTER, interpolatedPose)
        ) <=
        REEF_BOUNDARY.in(Meters)
      ) {
        return true;
      }
    }
    return false;
  }

  public Pose2d findNewTarget(
    Pose2d currTarget,
    Pose2d lastTarget,
    Pose2d currPose
  ) {
    // if we can go to the final goal without hitting it, just go there
    if (
      Math.abs(Utility.findDistanceBetweenPoses(currPose, m_finalGoal)) <=
        FINAL_APPROACH_DISTANCE.in(Meters) ||
      !willHitReef(currPose, m_finalGoal, DEFAULT_INTERPOLATION_PERCENTAGES)
    ) {
      m_currentTarget = m_finalGoal;
      return m_finalGoal;
    }

    // if (!isAtTarget(currTarget, currPose)) {
    //   return currTarget;
    // }

    Pose2d newTarget = interpolateTarget(currPose, m_finalGoal);
    if (willHitReef(currPose, newTarget, DEFAULT_INTERPOLATION_PERCENTAGES)) {
      newTarget = multiplyAway(currPose, newTarget, DirectionOfTravel.Left);
    }
    m_lastTarget = m_currentTarget;
    m_currentTarget = newTarget;
    Robot.elasticFieldManager.shooterFieldRep.setRobotPose(newTarget);
    return newTarget;
  }

  public void setFinalGoal(Pose2d finalGoal) {
    m_finalGoal = finalGoal;
  }

  public DirectionOfTravel findMainDirectionOfTravel(
    Pose2d currPose,
    Pose2d finalGoal
  ) {
    Pose2d currPoseToGoalDelta = getPoseDelta(currPose, finalGoal);
    double xDist = m_lastDoT == DirectionOfTravel.X
      ? currPoseToGoalDelta.getX() / 1.75
      : currPoseToGoalDelta.getX();
    double yDist = m_lastDoT == DirectionOfTravel.Y
      ? currPoseToGoalDelta.getY() / 1.75
      : currPoseToGoalDelta.getY();
    return xDist >= yDist ? DirectionOfTravel.Y : DirectionOfTravel.X;
  }

  /**
   * Pins the given pose to a specific distance away from the reef. The distance is specified by the IDEAL_DISTANCE_FROM_REEF constant.
   * <p>Only changes the non-pinned axis.
   * @param poseToPin The pose that you want to pin to the reef
   * @param pinnedDirection The axis (X or Y) that you dont want to change. Reccomend getting this from {@link DriveToReef#getDirectionOfTravel()}
   * @return
   */
  public Pose2d pinPoseToReef(
    Pose2d poseToPin,
    DirectionOfTravel pinnedDirection
  ) { // c^2 - a^2 = b^2 | c^2 - b^2 = a^2 | a = x, b = y, change whatever is NOT the pinned direction
    Pose2d poseToCenterDelta = getPoseDelta(REEF.CENTER, poseToPin);
    double a = poseToCenterDelta.getX(); // the side length of side a
    double b = poseToCenterDelta.getY(); // the side length of side b

    // pinned pose is relative to the center of the reef
    // this means that positive X is RIGHT, positive Y is UP, negative x is LEFT, negative y is DOWN.
    Pose2d pinnedPose = Pose2d.kZero;
    // because the reef boundary is a circle, theres multiple solutions we could use, so we have to find the best one we can.
    // we do this by pinning the pose in a specific way, dictated by the multiplier we apply to the pinned pose.
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
        // to try and find the best solution we can, we get two opposite solutions, and we figure out which one is closer to the goal, and use it.
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
        // to try and find the best solution we can, we get two opposite solutions, and we figure out which one is closer to the goal, and use it.
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
    // the pinned pose is relative to REEF.CENTER, so if we add that pose to it, it moves the origin back to blue origin, making it useable.
    Pose2d finalPose = pinnedPose.plus(Utility.poseToTransform(REEF.CENTER));
    // If the poseToPin is close to being pinned, the changed coord will be NaN due to the Math functions we call.
    // This is the easiest way to make sure we dont give back a NaN without doing preemptive calculations
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

  /**
   * Interpolates a new target based on the constant for interpolation distance.
   * @param currPose The current pose
   * @param goal The final goal
   * @return The interpolated pose
   */
  public Pose2d interpolateTarget(Pose2d currPose, Pose2d goal) {
    double dist = Utility.findDistanceBetweenPoses(currPose, goal);
    return currPose.interpolate(
      goal,
      (1 / dist) * INTERPOLATION_DISTANCE.in(Meters)
    );
  }

  /**
   * Uses some triangles to find a point that is tangential to the reef boundary so we can curve around it really nicely.<p>
   * If you need to see the drawing of it ask Max for it, cause I cant put this in a javadoc.
   * @param currPose Current alliance relative pose
   * @param interpTar interpolated target
   * @return A new target which will take the robot around the reef boundary
   */
  public Pose2d forceTangentToReefBoundary(Pose2d currPose, Pose2d interpTar) {
    // System.out.println("INTERPOLATED TARGET: " + interpTar + "\n-------------");
    // TA is a right triangle based on the center of the reef and current pose.
    // we need one of its angles to allow us to find an angle in triangle D as all the triangles together
    // make a right angle based on the current pose, so we can find any angle whose backing point is the current pose.
    Pose2d TASideLengths = getPoseDelta(REEF.CENTER, currPose);
    boolean isXShorterAxis = TASideLengths.getX() <= TASideLengths.getY();
    // triangle B isnt a right triangle, so we know the side lengths and have to calculate the angles we need based on that.
    double TBSideA = Utility.findDistanceBetweenPoses(currPose, REEF.CENTER);
    double TBSideB = Utility.findDistanceBetweenPoses(currPose, interpTar);
    double TBSideC = Utility.findDistanceBetweenPoses(REEF.CENTER, interpTar);

    // TB and TA share a side, with that side being the hypotenuse of TA, and TBSideA. sin(TAAngleA) = TAOpposite / TAHypotenuse;
    double TAAngleC = Math.asin(
      Units.radiansToRotations(
        Math.abs(isXShorterAxis ? TASideLengths.getX() : TASideLengths.getY()) /
        TBSideA
      )
    );
    TAAngleC += Math.copySign(Math.PI / 2, TAAngleC);
    // System.out.println(
    //   "TAAngC: " +
    //   TAAngleC +
    //   "\nTASidA: " +
    //   (isXShorterAxis ? TASideLengths.getX() : TASideLengths.getY()) +
    //   "\nTASidC: " +
    //   (!isXShorterAxis ? TASideLengths.getX() : TASideLengths.getY()) +
    //   "\n----------"
    // );
    // cos(C) = (a^2 + b^2 - c^2) / (2ab)
    double TBAngleC = Math.acos(
      (Math.pow(TBSideA, 2) + Math.pow(TBSideB, 2) - Math.pow(TBSideC, 2)) /
      (2 * TBSideA * TBSideB)
    );
    // cos(A) = (b^2 + c^2 - a^2) / (2bc)
    double TBAngleA = Math.acos(
      Units.radiansToRotations(
        (Math.pow(TBSideB, 2) + Math.pow(TBSideC, 2) - Math.pow(TBSideA, 2)) /
        (2 * TBSideB * TBSideC)
      )
    );
    // System.out.println(
    //   "TBSidA: " +
    //   TBSideA +
    //   "\nTBSidB: " +
    //   TBSideB +
    //   "\nTBSidC: " +
    //   TBSideC +
    //   "\nTBAngA: " +
    //   TBAngleA +
    //   "\nTBAngC: " +
    //   TBAngleC +
    //   "\n---------"
    // );
    // TC is also a non right triangle, with the points its lines are drawn between being the currPose, interpPose, and the desiredPose
    // we dont know the desired target, as thats what were finding, so we only know 2 sides, and have to find the rest.
    // TB and TC share a side, and have a straight line that makes up one of their sides, so TCAngleB + TBAngleA MUST equal 180.
    double TCAngleB = Units.degreesToRadians(180) - TBAngleA;
    double TCSideC = IDEAL_DISTANCE_FROM_REEF.in(Meters) - TBSideC;
    double TCSideB = Math.sqrt(
      Math.pow(TBSideA, 2) +
      Math.pow(TCSideC, 2) -
      (2 * TBSideB * TCSideC) * Math.cos(TCAngleB)
    );
    // cos(C) = (a^2 + b^2 - c^2) / (2ab)
    // from the side that TB and TC share, TCSideA = TBSideB
    double TCAngleC = Math.acos(
      Units.radiansToRotations(
        (Math.pow(TBSideB, 2) + Math.pow(TCSideB, 2) - Math.pow(TCSideC, 2)) /
        (2 * TBSideB * TCSideB)
      )
    );
    // System.out.println(
    //   "****** TCAngC ARC INPUT: " +
    //   Units.degreesToRadians(
    //     (Math.pow(TBSideB, 2) + Math.pow(TCSideB, 2) - Math.pow(TCSideC, 2)) /
    //     (2 * TBSideB * TCSideB)
    //   )
    // );
    // System.out.println(
    //   "TCAngB: " +
    //   TCAngleB +
    //   "\nTCAngC: " +
    //   TCAngleC +
    //   "\nTCSidB: " +
    //   TCSideB +
    //   "\nTCSidC: " +
    //   TCSideC +
    //   "\n----------"
    // );
    // now that we know all the angles with a backing point of the current pose, we can find TDAngleC by using them, as they all make a right angle when put together
    // TD is also a right triangle with its points being the desired target and current pose, so its sides are essentially a translation between the current pose and the desired target.
    double TDAngleC =
      Units.degreesToRadians(90) - TAAngleC - TBAngleC - TCAngleC;
    double TDSideB = TCSideB * Math.cos(TDAngleC);
    double TDSideA = TCSideB * Math.sin(TDAngleC);
    // System.out.println(
    //   "TDAngC: " +
    //   TDAngleC +
    //   "\nTDSidA: " +
    //   TDSideA +
    //   "\nTDSidB: " +
    //   TDSideB +
    //   "\n----------"
    // );
    // because of how the triangles are laid out, whichever side is longer on TD, is on the same axis as TASideC
    // and the shorther TD side is on the same axis as TASideA.
    Pose2d tangentialTarget = Pose2d.kZero;
    if (isXShorterAxis) {
      tangentialTarget = new Pose2d(
        currPose.getX() + TDSideB,
        currPose.getY() + TDSideA,
        currPose.getRotation()
      );
    } else {
      tangentialTarget = new Pose2d(
        currPose.getX() + TDSideB,
        currPose.getY() + TDSideA,
        currPose.getRotation()
      );
    }
    // we want to push past being strictly tangent, so we can more easily stay outside the boundry and curve better
    // Pose2d amountToPushPastTangent = getPoseDelta(currPose, tangentialTarget);
    // System.out.println("TAN TAR PRE PUSH : " + tangentialTarget);
    // tangentialTarget = new Pose2d(
    //   tangentialTarget.getX() + (amountToPushPastTangent.getX() * 0.15),
    //   tangentialTarget.getY() + (amountToPushPastTangent.getY() * 0.15),
    //   tangentialTarget.getRotation()
    // );
    // System.out.println("TAN TAR POST PUSH:" + tangentialTarget);

    return tangentialTarget;
  }

  public Pose2d multiplyAway(
    Pose2d currPose,
    Pose2d currTarget,
    DirectionOfTravel routeAroundReef
  ) {
    Pose2d target = currTarget;
    Pose2d relativeToReef = getPoseDelta(target, REEF.CENTER);
    double currPoseToTarDist = Math.abs(
      Utility.findDistanceBetweenPoses(currPose, target)
    );
    double tarToCenterDist = Math.abs(
      Utility.findDistanceBetweenPoses(target, REEF.CENTER)
    );
    double centerToCurrPoseDist = Math.abs(
      Utility.findDistanceBetweenPoses(currPose, REEF.CENTER)
    );
    double radiansToRotate = Math.acos(
      Units.radiansToRotations(
        (Math.pow(currPoseToTarDist, 2) +
          Math.pow(tarToCenterDist, 2) -
          Math.pow(centerToCurrPoseDist, 2)) /
        (2 * currPoseToTarDist * tarToCenterDist)
      )
    );
    Rotation2d roto = Rotation2d.fromRotations(radiansToRotate);
    Translation2d tarTranslation = relativeToReef
      .getTranslation()
      .rotateAround(Translation2d.kZero, roto);
    double multiplier =
      ((1 / tarToCenterDist) * IDEAL_DISTANCE_FROM_REEF.in(Meters));
    tarTranslation = new Translation2d(
      tarTranslation.getX() * (multiplier > 1 ? multiplier : 1),
      tarTranslation.getY() * (multiplier > 1 ? multiplier : 1)
    );

    tarTranslation.rotateAround(
      REEF.CENTER.getTranslation(),
      roto.unaryMinus()
    );
    target = new Pose2d(
      tarTranslation.getX() + REEF.CENTER.getX(),
      tarTranslation.getY() + REEF.CENTER.getY(),
      target.getRotation()
    );
    return target;
  }
}

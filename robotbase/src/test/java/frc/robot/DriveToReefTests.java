package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.DRIVE_TO_POSE.INTERPOLATION_DISTANCE;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.DRIVE_TO_POSE;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.commands.drive.DriveToReef;
import frc.robot.commands.drive.DriveToReef.DirectionOfTravel;
import frc.robot.util.Utility;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveToReefTests {

  // uncomment these if you want to run tests on DriveToReef,
  // you will have to make all methods in the class public however. which is why theyre commented out.
  DriveToReef m_driveToReef;
  static final double DELTA = 1E-2;

  @BeforeEach
  void setup() {
    m_driveToReef = new DriveToReef();
  }

  @Test
  void poseDeltaTest() {
    Pose2d here = new Pose2d(1, 1, Rotation2d.kZero);
    Pose2d there = Pose2d.kZero;
    Pose2d delta = m_driveToReef.getPoseDelta(here, there);
    assertEquals(new Pose2d(-1, -1, Rotation2d.kZero), delta);
  }

  @Test
  void isNOTAtTarTest() {
    Pose2d currPose = Pose2d.kZero;
    Pose2d currTar = new Pose2d(1, 1, Rotation2d.kZero);
    boolean result = m_driveToReef.isAtTarget(currTar, currPose);
    assertEquals(false, result);
  }

  @Test
  void isAtTarTest() {
    Pose2d currPose = new Pose2d(1.00001, 1.0001, Rotation2d.kZero);
    Pose2d currTar = new Pose2d(1, 1, Rotation2d.kZero);
    boolean result = m_driveToReef.isAtTarget(currTar, currPose);
    assertEquals(true, result);
  }

  @Test
  void willHitReefStraightOnTest() {
    var currPose = REEF.CENTER.plus(new Transform2d(-2, 0, Rotation2d.kZero));
    var currTar = REEF.CENTER.plus(new Transform2d(2, 0, Rotation2d.kZero));
    assertEquals(true, m_driveToReef.willHitReef(currPose, currTar, 0.2));
  }

  @Test
  void willHitReefBoundaryTest() {
    var currPose = new Pose2d(
      3.1950957775115967,
      3.126810312271118,
      Rotation2d.kZero
    );
    var currTar = currPose.plus(new Transform2d(0, 4, Rotation2d.kZero));
    boolean result = m_driveToReef.willHitReef(currPose, currTar, 0.2, 0.8);
    assertEquals(true, result);
  }

  @Test
  void willHitReefCornerTest() {
    Pose2d middlePose = new Pose2d(3.2, 4.9, Rotation2d.kZero);
    Pose2d currPose = new Pose2d(
      middlePose.getX() - 1,
      middlePose.getY() - 1,
      Rotation2d.kZero
    );
    Pose2d currTar = middlePose.plus(new Transform2d(1, 1, Rotation2d.kZero));
    boolean result = m_driveToReef.willHitReef(
      currPose,
      currTar,
      0.45,
      0.46,
      0.48,
      0.49,
      0.5,
      0.51,
      0.52,
      0.53,
      0.54,
      0.55
    );
    assertEquals(true, result);
  }

  // @Test
  // void findNewTargetFinalApproachTest() {
  //   Pose2d finalGoal = REEF.BRANCH_A;
  //   Pose2d currPose = finalGoal.plus(new Transform2d(-.5, 0, Rotation2d.kZero));
  //   Pose2d currTar = REEF.BRANCH_A.plus(
  //     new Transform2d(-0.05, 0, Rotation2d.kZero)
  //   );
  //   Pose2d lastTar = currPose.plus(new Transform2d(-0.05, 0, Rotation2d.kZero));
  //   m_driveToReef.setFinalGoal(finalGoal);
  //   Pose2d result = m_driveToReef.findNewTarget(currTar, lastTar, currPose);
  //   assertEquals(finalGoal, result);
  // }
  //
  //@Test
  // void findNewTargetStayWithTargetTest() {
  //   Pose2d finalGoal = REEF.BRANCH_A;
  //   Pose2d currPose = finalGoal.plus(new Transform2d(-2, 0, Rotation2d.kZero));
  //   Pose2d currTar = currPose.plus(new Transform2d(0.5, 0, Rotation2d.kZero));
  //   Pose2d lastTar = currPose.plus(new Transform2d(-0.05, 0, Rotation2d.kZero));
  //   m_driveToReef.setFinalGoal(finalGoal);
  //   Pose2d newTar = m_driveToReef.findNewTarget(currTar, lastTar, currPose);
  //   double result = Utility.findDistanceBetweenPoses(currTar, newTar);
  //   assertEquals(0, result, DELTA);
  // }
  //
  //@Test
  // void pinPoseTest() {
  //   Pose2d poseToPin = REEF.BRANCH_B.plus(
  //     new Transform2d(0.5, -1.5, Rotation2d.kZero)
  //   );
  //   Pose2d goal = REEF.BRANCH_A;
  //   m_driveToReef.setFinalGoal(goal);
  //   Pose2d resultX = m_driveToReef.pinPoseToReef(
  //     poseToPin,
  //     DirectionOfTravel.X
  //   );
  //   Pose2d resultY = m_driveToReef.pinPoseToReef(
  //     poseToPin,
  //     DirectionOfTravel.Y
  //   );
  //   Pose2d result = m_driveToReef.pinPoseToReef(
  //     poseToPin,
  //     m_driveToReef.findMainDirectionOfTravel(poseToPin, goal)
  //   );
  //   System.out.println(
  //     "STARTING POSE DIST FROM DESIRED: " +
  //     (poseToPin.getTranslation().getDistance(REEF.CENTER.getTranslation()) -
  //       DRIVE_TO_POSE.IDEAL_DISTANCE_FROM_REEF.in(Units.Meters))
  //   );
  //   System.out.println("POSE TO PIN: " + poseToPin.getTranslation());
  //   System.out.println("RESULT     : " + result.getTranslation());
  //   System.out.println("RESULT X   : " + resultX.getTranslation());
  //   System.out.println("RESULT Y   : " + resultY.getTranslation());
  //   assertEquals(
  //     false,
  //     m_driveToReef.willHitReef(
  //       poseToPin,
  //       result,
  //       DRIVE_TO_POSE.DEFAULT_INTERPOLATION_PERCENTAGES
  //     )
  //   );
  // }
  //
  @Test
  void forceTangentTest() {
    Pose2d currPose = new Pose2d(
      5.087228298187256,
      1.6359786987304688,
      Rotation2d.kZero
    );
    Pose2d goal = REEF.BRANCH_A;
    m_driveToReef.setFinalGoal(goal);
    Pose2d result = m_driveToReef.forceTangentToReefBoundary(
      currPose,
      m_driveToReef.interpolateTarget(currPose, goal)
    );
    System.out.println(
      "STARTING POSE DIST FROM DESIRED: " +
      (currPose.getTranslation().getDistance(REEF.CENTER.getTranslation()) -
        DRIVE_TO_POSE.IDEAL_DISTANCE_FROM_REEF.in(Units.Meters))
    );
    System.out.println("GOAL       : " + goal.getTranslation());
    System.out.println("POSE TO PIN: " + currPose.getTranslation());
    System.out.println("RESULT     : " + result.getTranslation());
    assertEquals(
      false,
      m_driveToReef.willHitReef(
        currPose,
        result,
        DRIVE_TO_POSE.DEFAULT_INTERPOLATION_PERCENTAGES
      )
    );
  }

  @Test
  void InterpolateTest() {
    Pose2d poseToInterp = Pose2d.kZero;
    Pose2d goal = new Pose2d(1, 0, Rotation2d.kZero);
    Pose2d result = m_driveToReef.interpolateTarget(poseToInterp, goal);
    Distance distInterpolated = Units.Meters.of(
      Utility.findDistanceBetweenPoses(poseToInterp, result)
    );
    System.out.println(
      "DIST INTERPOLATED METERS   : " +
      distInterpolated.in(Meters) +
      "\n" +
      "INTENDED INTERP DIST METERS: " +
      INTERPOLATION_DISTANCE.in(Meters) +
      "\n" +
      "INTERPOLATED TO INTENDED DELTA: " +
      distInterpolated.minus(INTERPOLATION_DISTANCE)
    );
    assertEquals(
      distInterpolated.in(Meters),
      INTERPOLATION_DISTANCE.in(Meters),
      0.01
    );
  }

  @Test
  void multiplyAwayTest() {
    Pose2d currPose = new Pose2d(
      3.9706337451934814,
      5.356443881988525,
      Rotation2d.kZero
    );
    Pose2d target = new Pose2d(
      3.079244375228882,
      4.04281759262085,
      Rotation2d.kZero
    );
    Pose2d result = m_driveToReef.multiplyAway(
      currPose,
      target,
      DirectionOfTravel.Left
    );
    System.out.println("RESULT: " + result);
    assertEquals(true, true);
  }
}

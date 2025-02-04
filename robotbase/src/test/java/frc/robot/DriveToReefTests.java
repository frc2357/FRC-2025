package frc.robot;
// class DriveToReefTests {
//   // uncomment these if you want to run tests on DriveToReef,
//   // you will have to make all methods in the class public however. which is why theyre commented out.
//   DriveToReef m_driveToReef;
//   static final double DELTA = 1E-2;
//   @BeforeEach
//   void setup() {
//     m_driveToReef = new DriveToReef();
//   }
//   @Test
//   void poseDeltaTest() {
//     Pose2d here = new Pose2d(1, 1, Rotation2d.kZero);
//     Pose2d there = Pose2d.kZero;
//     Pose2d delta = m_driveToReef.getPoseDelta(here, there);
//     assertEquals(new Pose2d(-1, -1, Rotation2d.kZero), delta);
//   }
//   @Test
//   void beyondTarTest() {
//     Pose2d currTar = new Pose2d(1, 1, Rotation2d.kZero);
//     Pose2d lastTar = Pose2d.kZero;
//     Pose2d currPose = Pose2d.kZero;
//     boolean isBeyondTar = m_driveToReef.isBeyondTarget(
//       currTar,
//       lastTar,
//       currPose
//     );
//     assertEquals(false, isBeyondTar);
//   }
//   @Test
//   void isNOTAtTarTest() {
//     Pose2d currPose = Pose2d.kZero;
//     Pose2d currTar = new Pose2d(1, 1, Rotation2d.kZero);
//     boolean result = m_driveToReef.isAtTarget(currTar, currPose);
//     assertEquals(false, result);
//   }
//   @Test
//   void isAtTarTest() {
//     Pose2d currPose = new Pose2d(1.00001, 1.0001, Rotation2d.kZero);
//     Pose2d currTar = new Pose2d(1, 1, Rotation2d.kZero);
//     boolean result = m_driveToReef.isAtTarget(currTar, currPose);
//     assertEquals(true, result);
//   }
//   @Test
//   void isFinalGoalTest() {
//     m_driveToReef.setFinalGoal(Pose2d.kZero);
//     assertEquals(true, m_driveToReef.isFinalGoal(Pose2d.kZero));
//   }
//   @Test
//   void willHitReefStraightOnTest() {
//     var currPose = REEF.CENTER.plus(new Transform2d(-2, 0, Rotation2d.kZero));
//     var currTar = REEF.CENTER.plus(new Transform2d(2, 0, Rotation2d.kZero));
//     assertEquals(true, m_driveToReef.willHitReef(currPose, currTar, 0.2));
//   }
//   @Test
//   void willHitReefBoundaryTest() {
//     var currPose = new Pose2d(
//       3.1950957775115967,
//       3.126810312271118,
//       Rotation2d.kZero
//     );
//     var currTar = currPose.plus(new Transform2d(0, 4, Rotation2d.kZero));
//     boolean result = m_driveToReef.willHitReef(currPose, currTar, 0.2, 0.8);
//     assertEquals(true, result);
//   }
//   @Test
//   void willHitReefCornerTest() {
//     Pose2d middlePose = new Pose2d(
//       3.2338643074035645,
//       4.927440166473389,
//       Rotation2d.kZero
//     );
//     Pose2d currPose = middlePose.plus(
//       new Transform2d(-1, -1, Rotation2d.kZero)
//     );
//     Pose2d currTar = middlePose.plus(new Transform2d(1, 1, Rotation2d.kZero));
//     boolean result = m_driveToReef.willHitReef(
//       currPose,
//       currTar,
//       0.45,
//       0.46,
//       0.48,
//       0.49,
//       0.5,
//       0.51,
//       0.52,
//       0.53,
//       0.54,
//       0.55
//     );
//     assertEquals(false, result);
//   }
//   @Test
//   void findNewTargetFinalApproachTest() {
//     Pose2d finalGoal = REEF.BRANCH_A;
//     Pose2d currPose = finalGoal.plus(new Transform2d(-.5, 0, Rotation2d.kZero));
//     Pose2d currTar = REEF.BRANCH_A.plus(
//       new Transform2d(-0.05, 0, Rotation2d.kZero)
//     );
//     Pose2d lastTar = currPose.plus(new Transform2d(-0.05, 0, Rotation2d.kZero));
//     m_driveToReef.setFinalGoal(finalGoal);
//     Pose2d result = m_driveToReef.findNewTarget(currTar, lastTar, currPose);
//     assertEquals(finalGoal, result);
//   }
//   @Test
//   void findNewTargetStayWithTargetTest() {
//     Pose2d finalGoal = REEF.BRANCH_A;
//     Pose2d currPose = finalGoal.plus(new Transform2d(-2, 0, Rotation2d.kZero));
//     Pose2d currTar = currPose.plus(new Transform2d(0.5, 0, Rotation2d.kZero));
//     Pose2d lastTar = currPose.plus(new Transform2d(-0.05, 0, Rotation2d.kZero));
//     m_driveToReef.setFinalGoal(finalGoal);
//     Pose2d newTar = m_driveToReef.findNewTarget(currTar, lastTar, currPose);
//     double result = Utility.findDistanceBetweenPoses(currTar, newTar);
//     assertEquals(0, result, DELTA);
//   }
//   // @Test // test to make sure it doesnt chuck the target way out of the reef
//   // void collisionAvoidanceTest() {
//   //   Pose2d currPose = new Pose2d(
//   //     4.1950957775115967,
//   //     2.126810312271118,
//   //     Rotation2d.kZero
//   //   );
//   //   Pose2d finalGoal = currPose.plus(new Transform2d(5, 6, Rotation2d.kZero));
//   //   Pose2d currTar = currPose.interpolate(finalGoal, 0.7);
//   //   Pose2d lastTar = currPose.plus(new Transform2d(-0.05, 0, Rotation2d.kZero));
//   //   m_driveToReef.setFinalGoal(currTar);
//   //   Pose2d newTar = m_driveToReef.findNewTarget(currTar, lastTar, currPose);
//   //   double result = Utility.findDistanceBetweenPoses(currTar, newTar);
//   //   System.out.println("RESULT:     " + result);
//   //   System.out.println("CURR POSE:  " + currPose);
//   //   System.out.println("CURR TAR:   " + currTar);
//   //   System.out.println("NEW TAR:    " + newTar);
//   //   System.out.println("FINAL GOAL: " + finalGoal);
//   //   newTar = m_driveToReef.pinPoseToReef(
//   //     newTar,
//   //     m_driveToReef.findMainDirectionOfTravel(currPose, finalGoal)
//   //   );
//   //   boolean willHitReef = m_driveToReef.willHitReef(
//   //     currPose,
//   //     newTar,
//   //     COLLISION_AVOIDANCE.DEFAULT_INTERPOLATION_PERCENTAGES
//   //   );
//   //   System.out.println("WILL HIT REEF: " + willHitReef);
//   //   assertEquals(false, willHitReef);
//   // }
//   @Test
//   void pinPoseTest() {
//     Pose2d poseToPin = REEF.BRANCH_B.plus(
//       new Transform2d(0.5, -1.5, Rotation2d.kZero)
//     );
//     Pose2d goal = REEF.BRANCH_A;
//     m_driveToReef.setFinalGoal(goal);
//     Pose2d resultX = m_driveToReef.pinPoseToReef(
//       poseToPin,
//       DirectionOfTravel.X
//     );
//     Pose2d resultY = m_driveToReef.pinPoseToReef(
//       poseToPin,
//       DirectionOfTravel.Y
//     );
//     Pose2d result = m_driveToReef.pinPoseToReef(
//       poseToPin,
//       m_driveToReef.findMainDirectionOfTravel(poseToPin, goal)
//     );
//     System.out.println(
//       "STARTING POSE DIST FROM DESIRED: " +
//       (poseToPin.getTranslation().getDistance(REEF.CENTER.getTranslation()) -
//         IDEAL_DISTANCE_FROM_REEF.in(Units.Meters))
//     );
//     System.out.println("POSE TO PIN: " + poseToPin.getTranslation());
//     System.out.println("RESULT     : " + result.getTranslation());
//     System.out.println("RESULT X   : " + resultX.getTranslation());
//     System.out.println("RESULT Y   : " + resultY.getTranslation());
//     assertEquals(
//       false,
//       m_driveToReef.willHitReef(
//         poseToPin,
//         result,
//         COLLISION_AVOIDANCE.DEFAULT_INTERPOLATION_PERCENTAGES
//       )
//     );
//   }
// }

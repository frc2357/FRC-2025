// package frc.robot;

// import static edu.wpi.first.units.Units.Meters;
// import static frc.robot.Constants.FIELD.REEF.BRANCH_A;
// import static org.junit.jupiter.api.Assertions.assertTrue;
// import static org.junit.jupiter.api.Assumptions.abort;

// import com.fasterxml.jackson.databind.deser.std.StdScalarDeserializer;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import frc.robot.Constants.FIELD_CONSTANTS;
// import frc.robot.Constants.ROBOT_CONFIGURATION;
// import frc.robot.subsystems.CameraManager;
// import frc.robot.util.Utility;
// import java.util.ArrayList;
// import java.util.random.RandomGenerator;
// import org.junit.jupiter.api.Test;

// public class PoseEstTests extends CameraManager {

//   CameraManager m_camManager;

//   @Test
//   // making absolutley sure nothing is wrong with the branch estimation at any translation or rotation
//   void branchEstimationTest() {
//     double desired = Math.hypot(
//       ROBOT_CONFIGURATION.FULL_LENGTH.div(2).in(Meters),
//       FIELD_CONSTANTS.BRANCH_TO_TAG_DIST.in(Meters)
//     );
//     var startingRotations = new ArrayList<Rotation2d>();
//     var startingTranslations = new ArrayList<Translation3d>();
//     for (double i = -180; i < 180; i += 0.01) {
//       startingRotations.add(Rotation2d.fromDegrees(i));
//     }
//     for (double i = -10; i < 10; i += 0.1) {
//       startingTranslations.add(new Translation3d(new Translation2d(i, i)));
//     }
//     ArrayList<Double> resultDists = new ArrayList<Double>();
//     ArrayList<Pose2d> failedResults = new ArrayList<Pose2d>();
//     double avrgDist = 0d;
//     for (Rotation2d rotation : startingRotations) {
//       for (Translation3d translation : startingTranslations) {
//         Pose3d startingPose = new Pose3d(translation, new Rotation3d(rotation));
//         Pose2d[] result =
//           this.calculateBranchPose(
//               startingPose,
//               startingPose.getRotation().toRotation2d()
//             );
//         Pose2d leftPose = result[0];
//         Pose2d rightPose = result[1];
//         double leftDist = leftPose
//           .getTranslation()
//           .getDistance(translation.toTranslation2d());
//         double rightDist = rightPose
//           .getTranslation()
//           .getDistance(translation.toTranslation2d());
//         double combinedDist = leftDist + rightDist;
//         combinedDist /= 2;
//         resultDists.add(combinedDist);
//         if (!Utility.isWithinTolerance(combinedDist, desired, 1E-12)) {
//           failedResults.add(startingPose.toPose2d());
//         }
//       }
//     }
//     for (Double dist : resultDists) {
//       avrgDist += dist;
//     }
//     avrgDist /= resultDists.size();
//     System.out.println(
//       "Average distance from \"tag\" pose = " +
//       avrgDist +
//       " | diff from expected " +
//       Math.abs(avrgDist - desired)
//     );
//     System.out.println("Failed Results Num = " + failedResults.size());

//     assertTrue(failedResults.isEmpty());
//   }
// }

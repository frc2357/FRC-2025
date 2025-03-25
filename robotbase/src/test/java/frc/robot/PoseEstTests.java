package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.FIELD_CONSTANTS;
import frc.robot.Constants.ROBOT_CONFIGURATION;
import frc.robot.subsystems.CameraManager;
import frc.robot.util.Utility;
import java.util.ArrayList;
import org.junit.jupiter.api.Test;

public class PoseEstTests extends CameraManager {

  CameraManager m_camManager;

  @Test
  // making absolutley sure nothing is wrong with the branch estimation at any translation or rotation
  void branchEstimationTest() {
    double desired = Math.hypot(
      ROBOT_CONFIGURATION.FULL_LENGTH.div(2).in(Meters),
      FIELD_CONSTANTS.BRANCH_TO_TAG_DIST.in(Meters)
    );

    ArrayList<Pose2d> failedResults = new ArrayList<Pose2d>();
    double avrgDist = 0d;
    int timesRan = 0;
    for (double rot = -2 * Math.PI; rot <= 2 * Math.PI; rot += 0.0005) {
      for (double translation = -10; translation < 10; translation += 0.1) {
        Pose2d startingPose = new Pose2d(
          translation,
          translation,
          Rotation2d.fromRadians(rot)
        );
        Pose2d[] result =
          this.calculateBranchPose(startingPose, startingPose.getRotation());
        Pose2d leftPose = result[0];
        Pose2d rightPose = result[1];
        double leftDist = leftPose
          .getTranslation()
          .getDistance(startingPose.getTranslation());
        double rightDist = rightPose
          .getTranslation()
          .getDistance(startingPose.getTranslation());
        double combinedDist = leftDist + rightDist;
        combinedDist /= 2;
        avrgDist += combinedDist;
        timesRan++;
        if (!Utility.isWithinTolerance(combinedDist, desired, 1E-12)) {
          failedResults.add(startingPose);
        }
      }
    }
    avrgDist /= timesRan;
    System.out.println(
      "Average distance from \"tag\" pose = " +
      avrgDist +
      " | diff from expected " +
      Math.abs(avrgDist - desired)
    );
    System.out.println("Times ran = " + timesRan);
    System.out.println("Failed Results Num = " + failedResults.size());

    assertTrue(
      Utility.isWithinTolerance(avrgDist, desired, 1E-10) &&
      failedResults.isEmpty()
    );
  }
}

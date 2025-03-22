package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.CameraManager;
import frc.robot.util.Utility;
import org.junit.jupiter.api.Test;

public class PoseEstTests extends CameraManager {

  CameraManager m_camManager;

  @Test
  void branchEstimationTest() {
    var startingPose = new Pose3d(
      4.058923244476318,
      3.293692111968994,
      0,
      new Rotation3d(Utility.invert(Rotation2d.fromRadians(1.0065281615966604)))
    );
    var result =
      this.calculateBranchPose(
          new Pose3d(
            4.058923244476318,
            3.293692111968994,
            0,
            new Rotation3d(
              Utility.invert(Rotation2d.fromRadians(1.0065281615966604))
            )
          )
        );
    var leftPose = result.getSecond();
    var rightPose = result.getFirst();
    System.out.println("LEFT BRANCH - " + leftPose);
    System.out.println("RIGHT BRANCH - " + rightPose);
    System.out.println(
      "LEFT DIFF X = " + (leftPose.minus(startingPose.toPose2d()).getX())
    );
    System.out.println(
      "LEFT DIFF Y = " + (leftPose.minus(startingPose.toPose2d()).getY())
    );
  }
}

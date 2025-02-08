package frc.robot;

import static frc.robot.Constants.DRIVE_TO_POSE.DEFAULT_INTERPOLATION_PERCENTAGES;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.util.CollisionDetection;
import org.junit.jupiter.api.Test;

public class CollisionDetectionTests {

  @Test
  void willHitReefStraightOnTest() {
    var currPose = REEF.CENTER.plus(new Transform2d(-2, 0, Rotation2d.kZero));
    var currTar = REEF.CENTER.plus(new Transform2d(2, 0, Rotation2d.kZero));
    assertEquals(
      true,
      CollisionDetection.willHitReef(
        currPose,
        currTar,
        DEFAULT_INTERPOLATION_PERCENTAGES
      )
    );
  }

  @Test
  void willHitReefBoundaryTest() {
    var currPose = new Pose2d(
      3.1950957775115967,
      3.126810312271118,
      Rotation2d.kZero
    );
    var currTar = currPose.plus(new Transform2d(0, 4, Rotation2d.kZero));
    boolean result = CollisionDetection.willHitReef(
      currPose,
      currTar,
      0.2,
      0.8
    );
    assertEquals(false, result);
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
    boolean result = CollisionDetection.willHitReef(
      currPose,
      currTar,
      DEFAULT_INTERPOLATION_PERCENTAGES
    );
    assertEquals(false, result);
  }
}

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.commands.drive.DriveToReef;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class DriveLockReefTests {

  private Rotation2d calculate(Pose2d currentPose, Pose2d centerPose) {
    Pose2d relativePose = centerPose.relativeTo(currentPose);
    return new Rotation2d(relativePose.getX(), relativePose.getY());
  }

  @Test
  void squareTest() {
    var currentPose = new Pose2d(1, 1, Rotation2d.kZero);
    var centerPose = new Pose2d(2, 2, Rotation2d.kZero);

    assertEquals(
      new Rotation2d(Math.PI / 4),
      calculate(currentPose, centerPose)
    );
  }

  @Test
  void arbituraryPointTest() {
    var currentPose = new Pose2d(1.32, 1.53, Rotation2d.kZero);
    var centerPose = new Pose2d(6.37, 3.26, Rotation2d.kZero);

    assertEquals(
      new Rotation2d(0.330044203096),
      calculate(currentPose, centerPose)
    );
  }

  @Test
  void negativeAngleTest() {
    var currentPose = new Pose2d(1.32, 1.53, Rotation2d.kZero);
    var centerPose = new Pose2d(-.77, -3.83, Rotation2d.kZero);

    assertEquals(
      new Rotation2d(-1.94258762436),
      calculate(currentPose, centerPose)
    );
  }

  @Test
  void zeroDeltaX() {
    var currentPose = new Pose2d(1, 1.53, Rotation2d.kZero);
    var centerPose = new Pose2d(1, -3.83, Rotation2d.kZero);

    assertEquals(
      new Rotation2d(4.71238898038),
      calculate(currentPose, centerPose)
    );
  }

  @Test
  void zeroDeltaY() {
    var currentPose = new Pose2d(1.32, 2, Rotation2d.kZero);
    var centerPose = new Pose2d(-.77, 2, Rotation2d.kZero);

    assertEquals(
      new Rotation2d(3.14159265359),
      calculate(currentPose, centerPose)
    );
  }
}

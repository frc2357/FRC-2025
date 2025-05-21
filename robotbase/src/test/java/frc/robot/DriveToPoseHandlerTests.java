package frc.robot;

import static edu.wpi.first.units.Units.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.DRIVE_TO_POSE;
import frc.robot.commands.drive.DriveToPoseHandler;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveToPoseHandlerTests extends DriveToPoseHandler {

  static final double DELTA = 1E-2;

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @BeforeEach
  void setup() {}

  @Test
  void isNOTAtTarTest() {
    Pose2d currPose = Pose2d.kZero;
    Pose2d currTar = new Pose2d(4, 4, Rotation2d.kZero);
    boolean result = super.isAtTarget(
      currTar,
      currPose,
      DRIVE_TO_POSE.FINAL_APPROACH_TOLERANCE_POSE
    );
    assertEquals(false, result);
  }

  @Test
  void isAtTarTest() {
    Pose2d currPose = new Pose2d(1.00001, 1.0001, Rotation2d.kZero);
    Pose2d currTar = new Pose2d(1, 1, Rotation2d.kZero);
    boolean result = super.isAtTarget(
      currTar,
      currPose,
      DRIVE_TO_POSE.FINAL_APPROACH_TOLERANCE_POSE
    );
    assertEquals(true, result);
  }

  @Test
  void InterpolateTest() {
    Pose2d poseToInterp = Pose2d.kZero;
    Pose2d goal = new Pose2d(1, 0, Rotation2d.kZero);
    Pose2d result = super.interpolateTarget(poseToInterp, goal);
    Distance distInterpolated = Units.Meters.of(
      poseToInterp.getTranslation().getDistance(result.getTranslation())
    );
    System.out.println("Dist = " + distInterpolated);
    assertEquals(
      distInterpolated.in(Meters),
      DRIVE_TO_POSE.INTERPOLATION_DISTANCE.in(Meters),
      0.01
    );
  }
}

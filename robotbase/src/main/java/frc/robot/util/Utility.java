package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.FIELD.REEF;

public class Utility {

  private static final Angle m_angle180Degrees = Units.Degrees.of(180);

  public static boolean isWithinTolerance(
    double currentValue,
    double targetValue,
    double tolerance
  ) {
    return Math.abs(currentValue - targetValue) <= tolerance;
  }

  public static boolean isWithinTolerance(
    Distance currentValue,
    Distance targetValue,
    Distance tolerance
  ) {
    return isWithinTolerance(
      currentValue.in(Inches),
      targetValue.in(Inches),
      tolerance.in(Inches)
    );
  }

  public static boolean isWithinTolerance(
    Angle currentValue,
    Angle targetValue,
    Angle tolerance
  ) {
    return isWithinTolerance(
      currentValue.in(Rotations),
      targetValue.in(Rotations),
      tolerance.in(Rotations)
    );
  }

  public static boolean isWithinTolerance(
    Translation2d currentValue,
    Translation2d targetValue,
    Translation2d tolerance
  ) {
    return (
      isWithinTolerance(
        currentValue.getX(),
        targetValue.getX(),
        tolerance.getX()
      ) &&
      isWithinTolerance(
        currentValue.getY(),
        targetValue.getY(),
        tolerance.getY()
      )
    );
  }

  public static double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    }
    return input;
  }

  /**
   * From:
   * https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/util/GeomUtil.java
   * Creates a pure translating transform
   *
   * @param x The x component of the translation
   * @param y The y component of the translation
   * @return The resulting transform
   */
  public static Transform2d translationToTransform(double x, double y) {
    return new Transform2d(new Translation2d(x, y), Rotation2d.kZero);
  }

  public static Transform2d translationToTransform(Translation2d translation) {
    return new Transform2d(translation, Rotation2d.kZero);
  }

  public static Transform2d poseToTransform(Pose2d pose) {
    return new Transform2d(pose.getTranslation(), pose.getRotation());
  }

  /**
   * Performs the Pythagorean Theorem on 2 numbers. Returns c
   * @param a A number that is not null or NaN
   * @param b A number that is not null or NaN
   * @return The square root of a^2 + b^2
   */
  public static double findHypotenuse(double a, double b) {
    // a^2 + b^2 = c^2
    return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
  }

  /**
   * Finds the distance between any 2 given points. Finds how far "there" is from "here"
   * @param here The point that will become the origin (0,0)
   * @param there The point that is being used to find the distance from "here"
   * @return The distance between the 2 points in meters, it may also be negative.
   */
  public static double findDistanceBetweenPoses(Pose2d here, Pose2d there) {
    return here.getTranslation().getDistance(there.getTranslation());
  }

  public static Rotation2d invert(Rotation2d rotation) {
    return rotation.plus(Rotation2d.k180deg);
  }

  public static Angle invert(Angle angle) {
    return angle.plus(m_angle180Degrees);
  }

  public static AprilTagFieldLayout makeHomeField() {
    double correctDistFromLeftInches = 14.25;
    double[] tagDists = { 16.125, 14.25, 14.125, 16.4375, 16.375, 16.375 };
    var normalLayout = AprilTagFieldLayout.loadField(
      AprilTagFields.k2025ReefscapeAndyMark
    );
    var poses = normalLayout.getTags();
    for (int i = 0; i < REEF.BLUE_REEF_TAGS.length; i++) {
      int tag = REEF.BLUE_REEF_TAGS[i];
      var tagPose = poses.get(tag - 1).pose;
      var correctionTransform = new Transform2d(
        Units.Inches.zero(),
        Units.Inches.of(correctDistFromLeftInches - tagDists[i]),
        Rotation2d.kZero
      );
      var correctedTagPose = tagPose.transformBy(
        new Transform3d(correctionTransform)
      );
      poses.set(tag - 1, new AprilTag(tag, correctedTagPose));
    }
    return new AprilTagFieldLayout(
      poses,
      normalLayout.getFieldLength(),
      normalLayout.getFieldWidth()
    );
  }
}

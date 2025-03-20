package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Utility {

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
}

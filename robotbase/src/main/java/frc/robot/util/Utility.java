package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Utility {

  public static boolean isWithinTolerance(
    double currentValue,
    double targetValue,
    double tolerance
  ) {
    return Math.abs(currentValue - targetValue) <= tolerance;
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
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }

  /**
   * Performs the Pythagorean Theorem on 2 numbers. Returns c
   * @param a A number that is not null or NaN
   * @param b A number that is not null or NaN
   * @return The square root of a^2 + b^2
   */
  public static double pythagorean(double a, double b) {
    // a^2 + b^2 = c^2
    return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
  }
}

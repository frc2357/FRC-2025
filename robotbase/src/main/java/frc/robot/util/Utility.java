package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
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
    var distanceBetweenPoints = there.relativeTo(here); // gets a pose for the distance of "there" from here
    return findHypotenuse(
      // gets the hypotenuse of the pose, which now has an origin of "here"
      distanceBetweenPoints.getX(),
      distanceBetweenPoints.getY()
    ); // this essentially measures how far away "there" is from "here"
  }
}

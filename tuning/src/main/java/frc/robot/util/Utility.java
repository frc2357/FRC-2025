package frc.robot.util;

public class Utility {

  public static boolean isWithinTolerance(
      double currentValue, double targetValue, double tolerance) {
    return Math.abs(currentValue - targetValue) <= tolerance;
  }

  public static double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    }
    return input;
  }
}

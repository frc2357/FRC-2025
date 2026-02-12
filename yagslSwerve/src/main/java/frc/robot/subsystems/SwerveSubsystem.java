package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  double maximumSpeed = Units.feetToMeters(4.5);
  File swerveJsonDirectory = new File(
    Filesystem.getDeployDirectory(),
    "swerve"
  );
  SwerveDrive swerveDrive;

  public SwerveSubsystem() {
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(
        maximumSpeed
      );
    } catch (IOException e) {
      e.printStackTrace();
    }

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(
    DoubleSupplier translationX,
    DoubleSupplier translationY,
    DoubleSupplier angularRotationX
  ) {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(
        SwerveMath.scaleTranslation(
          new Translation2d(
            translationX.getAsDouble() *
            swerveDrive.getMaximumChassisVelocity(),
            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
          ),
          0.8
        ),
        Math.pow(angularRotationX.getAsDouble(), 3) *
        swerveDrive.getMaximumChassisAngularVelocity(),
        true,
        true
      );
    });
  }
}

package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DRIVE_TO_YAW_PITCH;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Utility;
import java.util.Optional;
import java.util.function.Supplier;

public class DriveToYawPitch extends Command {

  private static final double m_speedAt12VoltsMPS =
    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  private ProfiledPIDController m_driveController;
  private ProfiledPIDController m_thetaController;

  private Supplier<Optional<Translation2d>> m_yawPitchSupplier;
  private Supplier<Pose2d> m_targetSupplier;

  /**
   *
   * @param yawPitchSupplier Returns a translation of an april tag target where the x is the yaw, and pitch is the y
   * @param targetSupplier Returns the target yaw, pitch, and field-relative robot rotational position
   */
  public DriveToYawPitch(
    Supplier<Optional<Translation2d>> yawPitchSupplier,
    Supplier<Pose2d> targetSupplier
  ) {
    addRequirements(Robot.swerve);

    m_yawPitchSupplier = yawPitchSupplier;
    m_driveController = DRIVE_TO_YAW_PITCH.DRIVE_CONTROLLER;
    m_thetaController = DRIVE_TO_YAW_PITCH.THETA_CONTROLLER;
  }

  @Override
  public void initialize() {
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Optional<Translation2d> current = m_yawPitchSupplier.get();
    if (current.isEmpty()) {
      cancel();
      return;
    }

    Pose2d target = m_targetSupplier.get();

    // Not sure if initial velocity still needs rotated by error or not.
    m_driveController.reset(
      new TrapezoidProfile.State(
        current.get().getDistance(target.getTranslation()),
        -new Translation2d(
          Robot.swerve.getRobotVelocity().dx,
          Robot.swerve.getRobotVelocity().dy
        )
          .rotateBy(
            target.getTranslation().minus(current.get()).getAngle().unaryMinus()
          )
          .getX()
      )
    );

    m_thetaController.reset(
      Robot.swerve.getFieldRelativePose2d().getRotation().getRadians()
    );
  }

  @Override
  public void execute() {
    Optional<Translation2d> current = m_yawPitchSupplier.get();
    Pose2d target = m_targetSupplier.get();

    Translation2d driveVelocity = new Translation2d(
      Robot.driverControls.getY() * m_speedAt12VoltsMPS,
      Robot.driverControls.getX() * m_speedAt12VoltsMPS
    );
    double thetaVelocity =
      Robot.driverControls.getRotation() *
      Constants.SWERVE.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

    if (
      driveVelocity.equals(Translation2d.kZero) &&
      thetaVelocity == 0 &&
      current.isPresent()
    ) {
      // Calculate drive speed
      double currentDistance = current
        .get()
        .getDistance(target.getTranslation());
      double driveErrorAbs = currentDistance;
      double driveVelocityScalar = m_driveController.calculate(
        driveErrorAbs,
        0.0
      );
      if (m_driveController.atGoal()) driveVelocityScalar = 0.0;

      // Calculate theta speed
      thetaVelocity = m_thetaController.calculate(
        Robot.swerve.getFieldRelativePose2d().getRotation().getRadians(),
        target.getRotation().getMeasure().in(Radians)
      );
      if (m_thetaController.atGoal()) thetaVelocity = 0.0;

      // Command speeds
      driveVelocity = new Pose2d(
        Translation2d.kZero,
        current.get().minus(target.getTranslation()).getAngle()
      )
        .transformBy(Utility.translationToTransform(driveVelocityScalar, 0.0))
        .getTranslation();
    }

    Robot.swerve.driveRobotRelative(
      driveVelocity.getX(),
      driveVelocity.getY(),
      thetaVelocity
    );
  }

  @Override
  public boolean isFinished() {
    return m_driveController.atGoal() && m_thetaController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {}
}

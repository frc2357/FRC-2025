package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DRIVE_TO_POSE;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Utility;
import java.util.function.Function;

public class DriveToPose extends Command {

  private Function<Pose2d, Pose2d> m_targetPoseFunction;

  private ProfiledPIDController m_driveController;
  private ProfiledPIDController m_thetaController;

  private static final double m_speedAt12VoltsMPS =
    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  public DriveToPose(Function<Pose2d, Pose2d> targetPoseFunction) {
    addRequirements(Robot.swerve);

    m_targetPoseFunction = targetPoseFunction;
    m_driveController = DRIVE_TO_POSE.AUTO_ALIGN_DRIVE_CONTROLLER;
    m_thetaController = DRIVE_TO_POSE.AUTO_ALIGN_THETA_CONTROLLER;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = Robot.swerve.getAllianceRelativePose2d();
    Pose2d targetPose = m_targetPoseFunction.apply(currentPose);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_driveController.reset(
      new TrapezoidProfile.State(
        currentPose.getTranslation().getDistance(targetPose.getTranslation()),
        -new Translation2d(
          Robot.swerve.getFieldVelocity().dx,
          Robot.swerve.getFieldVelocity().dy
        )
          .rotateBy(
            targetPose
              .getTranslation()
              .minus(currentPose.getTranslation())
              .getAngle()
              .unaryMinus()
          )
          .getX()
      )
    );

    m_thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = Robot.swerve.getAllianceRelativePose2d();
    Pose2d targetPose = m_targetPoseFunction.apply(currentPose);

    Translation2d driveVelocity = new Translation2d(
      Robot.driverControls.getY() * m_speedAt12VoltsMPS,
      Robot.driverControls.getX() * m_speedAt12VoltsMPS
    );
    double thetaVelocity =
      Robot.driverControls.getRotation() *
      Constants.SWERVE.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

    if (driveVelocity.equals(Translation2d.kZero) && thetaVelocity == 0) {
      // Calculate drive speed
      double currentDistance = currentPose
        .getTranslation()
        .getDistance(targetPose.getTranslation());
      double driveErrorAbs = currentDistance;
      double driveVelocityScalar = m_driveController.calculate(
        driveErrorAbs,
        0.0
      );
      if (m_driveController.atGoal()) driveVelocityScalar = 0.0;

      // Calculate theta speed
      thetaVelocity = m_thetaController.calculate(
        currentPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians()
      );
      if (m_thetaController.atGoal()) thetaVelocity = 0.0;

      // Command speeds
      driveVelocity = new Pose2d(
        Translation2d.kZero,
        currentPose
          .getTranslation()
          .minus(targetPose.getTranslation())
          .getAngle()
      )
        .transformBy(Utility.translationToTransform(driveVelocityScalar, 0.0))
        .getTranslation();
    }

    // System.out.println("ROTO = " + m_rotoOutput);
    Robot.swerve.driveFieldRelative(
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

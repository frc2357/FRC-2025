package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.DRIVE_TO_POSE.FINAL_APPROACH_TOLERANCE_POSE;

import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
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
import org.ejml.data.MatrixType;

public class DriveToPose extends Command {

  private Function<Pose2d, Pose2d> m_targetPoseFunction;

  private ProfiledPIDController m_driveController;
  private ProfiledPIDController m_thetaController;

  private Pose2d m_targetPose;

  private static final double m_speedAt12VoltsMPS =
    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  public DriveToPose(Function<Pose2d, Pose2d> targetPoseFunction) {
    addRequirements(Robot.swerve);

    m_targetPoseFunction = targetPoseFunction;
    m_driveController = DRIVE_TO_POSE.DRIVE_CONTROLLER;
    m_thetaController = DRIVE_TO_POSE.THETA_CONTROLLER;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = Robot.swerve.getFieldRelativePose2d();
    m_targetPose = m_targetPoseFunction.apply(currentPose);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    resetControllers(currentPose, m_targetPose);
  }

  @Override
  public void execute() {
    Pose2d currentPose = Robot.swerve.getFieldRelativePose2d();
    Pose2d newTargetPose = Robot.swerve.makePoseAllianceRelative(
      m_targetPoseFunction.apply(currentPose)
    );
    if (
      !Utility.isWithinTolerance(
        m_targetPose.getTranslation(),
        newTargetPose.getTranslation(),
        FINAL_APPROACH_TOLERANCE_POSE.getTranslation()
      )
    ) {
      m_targetPose = newTargetPose;
      resetControllers(currentPose, m_targetPose);
    }
    Translation2d driveVelocity = new Translation2d(
      Robot.driverControls.getY() * m_speedAt12VoltsMPS,
      Robot.driverControls.getX() * m_speedAt12VoltsMPS
    );

    double thetaVelocity =
      Robot.driverControls.getRotation() *
      Constants.SWERVE.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

    if (driveVelocity.equals(Translation2d.kZero) && thetaVelocity == 0) {
      // Calculate drive speed
      double currentDistanceFromTarPose = currentPose
        .getTranslation()
        .getDistance(m_targetPose.getTranslation());
      double driveErrorAbs = currentDistanceFromTarPose;
      double driveVelocityScalar = m_driveController.calculate(
        driveErrorAbs,
        0.0
      );
      if (m_driveController.atGoal()) driveVelocityScalar = 0.0;

      // Calculate theta speed
      thetaVelocity = m_thetaController.calculate(
        currentPose.getRotation().getRadians(),
        m_targetPose.getRotation().getRadians()
      );
      if (m_thetaController.atGoal()) thetaVelocity = 0.0;

      // Command speeds
      // creates a pose that only has an angle pointing from the current pose to the target
      driveVelocity = new Pose2d(
        Translation2d.kZero,
        currentPose
          .getTranslation()
          .minus(m_targetPose.getTranslation())
          .getAngle()
      )
        // then pushes that pose by the driveVelocityScalar.
        // this pushes it towards the goal, as the transform used only has an X component.
        .transformBy(Utility.translationToTransform(driveVelocityScalar, 0.0))
        // then uses that translation and its X & Y components as the drive velocities.
        .getTranslation();
    }

    Robot.swerve.driveFieldRelative(
      driveVelocity.getX(),
      driveVelocity.getY(),
      thetaVelocity,
      ForwardPerspectiveValue.BlueAlliance
    );
  }

  private void resetControllers(Pose2d currPose, Pose2d targetPose) {
    m_driveController.reset(
      new TrapezoidProfile.State(
        currPose.getTranslation().getDistance(targetPose.getTranslation()),
        -new Translation2d(
          Robot.swerve.getFieldRelativeRobotVelocity().dx,
          Robot.swerve.getFieldRelativeRobotVelocity().dy
        )
          .rotateBy(
            targetPose
              .getTranslation()
              .minus(currPose.getTranslation())
              .getAngle()
              .unaryMinus()
          )
          .getX()
      )
    );

    m_thetaController.reset(currPose.getRotation().getRadians());
  }

  @Override
  public boolean isFinished() {
    // return m_driveController.atGoal() && m_thetaController.atGoal();
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}

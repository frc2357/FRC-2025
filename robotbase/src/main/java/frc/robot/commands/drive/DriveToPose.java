package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.DRIVE_TO_POSE.FINAL_APPROACH_TOLERANCE_POSE;

import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DRIVE_TO_POSE;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Utility;
import java.util.function.Function;

public class DriveToPose extends Command {

  private Function<Pose2d, Pose2d> m_targetPoseFunction;

  private ProfiledPIDController m_driveController =
    DRIVE_TO_POSE.DRIVE_CONTROLLER;
  private ProfiledPIDController m_thetaController =
    DRIVE_TO_POSE.THETA_CONTROLLER;

  private Pose2d m_targetPose;

  private Pose2d m_startingPose;

  private boolean m_isDriverDriving = false;

  private static final double m_speedAt12VoltsMPS =
    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  public DriveToPose(Function<Pose2d, Pose2d> targetPoseFunction) {
    addRequirements(Robot.swerve);

    m_targetPoseFunction = targetPoseFunction;
  }

  @Override
  public void initialize() {
    Pose2d currentPose = Robot.swerve.getFieldRelativePose2d();
    m_startingPose = currentPose;
    m_targetPose = m_targetPoseFunction.apply(currentPose);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_driveController.setConstraints(DRIVE_TO_POSE.DRIVE_DEFAULT_CONSTRAINTS);

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
        FINAL_APPROACH_TOLERANCE_POSE.getTranslation().div(2)
      )
    ) {
      m_targetPose = newTargetPose;
    }
    Translation2d driveVelocity = new Translation2d(
      Robot.driverControls.getY() * m_speedAt12VoltsMPS,
      Robot.driverControls.getX() * m_speedAt12VoltsMPS
    );

    double thetaVelocity =
      Robot.driverControls.getRotation() *
      Constants.SWERVE.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);

    if (driveVelocity.equals(Translation2d.kZero) && thetaVelocity == 0) {
      if (m_isDriverDriving) {
        resetControllers(currentPose, m_targetPose);
        m_isDriverDriving = false;
      }
      // Calculate drive speed
      // we find our current distance and our targets distance from "zero"
      // we set the "zero" as our starting pose.
      double currDistFromStartPose = m_startingPose
        .getTranslation()
        .getDistance(currentPose.getTranslation());
      double startDistToTarPose = m_startingPose
        .getTranslation()
        .getDistance(m_targetPose.getTranslation());
      // we use our distance from the zero as our position, and our targets distance from zero as our goal
      double driveVelocityScalar = m_driveController.calculate(
        currDistFromStartPose,
        startDistToTarPose
      );
      if (m_driveController.atGoal()) driveVelocityScalar = 0.0;

      // Calculate theta speed
      thetaVelocity = m_thetaController.calculate(
        currentPose.getRotation().getRadians(),
        m_targetPose.getRotation().getRadians()
      );
      if (m_thetaController.atGoal()) thetaVelocity = 0.0;

      // Command speeds
      driveVelocity =
        // makes a pose that is just the angle from the current pose to the goal
        new Pose2d(
          Translation2d.kZero,
          currentPose
            .getTranslation()
            .minus(m_targetPose.getTranslation())
            .getAngle()
        )
          // then pushes that pose by the driveVelocityScalar.
          // this pushes it towards the goal, as the transform used only has an X component.
          .transformBy(
            Utility.translationToTransform(-driveVelocityScalar, 0.0)
          )
          // because the pose being transformed was just pointing towards the goal, we have applied the PID output to move towards the goal.
          // the speeds will be shared among the X & Y components depending on the angle.
          .getTranslation();
    } else {
      m_isDriverDriving = true;
    }

    Robot.swerve.driveFieldRelative(
      driveVelocity.getX(),
      driveVelocity.getY(),
      thetaVelocity,
      ForwardPerspectiveValue.BlueAlliance
    );
  }

  private void resetControllers(Pose2d currPose, Pose2d targetPose) {
    Twist2d robotVelocity = Robot.swerve.getFieldRelativeRobotVelocity();
    m_driveController.reset(
      new TrapezoidProfile.State(
        0,
        -new Translation2d(robotVelocity.dx, robotVelocity.dy)
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
    m_startingPose = currPose;
  }

  public void setDriveConstraints(Constraints newConstraints) {
    m_driveController.setConstraints(newConstraints);
  }

  public Constraints getDriveConstraints() {
    return m_driveController.getConstraints();
  }

  public void setThetaConstraints(Constraints newConstraints) {
    m_thetaController.setConstraints(newConstraints);
  }

  public Constraints getThetaConstraints() {
    return m_thetaController.getConstraints();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}

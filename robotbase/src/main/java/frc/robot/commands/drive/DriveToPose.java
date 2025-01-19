package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DRIVE_TO_POSE;
import frc.robot.Robot;
import frc.robot.util.Utility;

public class DriveToPose extends Command {

  private Pose2d m_targetPose;
  private Pose2d m_currentPose;

  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_rotationController;

  public DriveToPose(Pose2d targetPose) {
    addRequirements(Robot.swerve);
    m_targetPose = targetPose;

    m_xController = DRIVE_TO_POSE.VISION_X_TRANSLATION_PID_CONTROLLER;
    m_yController = DRIVE_TO_POSE.VISION_Y_TRANSLATION_PID_CONTROLLER;
    m_rotationController = DRIVE_TO_POSE.PIGEON_ROTATION_PID_CONTROLLER;
  }

  @Override
  public void initialize() {
    m_xController.setTolerance(DRIVE_TO_POSE.WAYPOINT_X_TOLERANCE.in(Meters));
    m_yController.setTolerance(DRIVE_TO_POSE.WAYPOINT_Y_TOLERANCE.in(Meters));
    m_rotationController.setTolerance(
      DRIVE_TO_POSE.WAYPOINT_ROTATION_TOLERANCE.in(Radians)
    );

    m_xController.setSetpoint(m_targetPose.getX());
    m_yController.setSetpoint(m_targetPose.getY());
    m_rotationController.setSetpoint(m_targetPose.getRotation().getRadians());

    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

    m_xController.reset();
    m_yController.reset();
    m_rotationController.reset();
    System.out.println(
      "****\nSTART DRIVE TO POSE\nPOSE AT START: " + Robot.swerve.getPose2d()
    );
  }

  @Override
  public void execute() {
    m_currentPose = Robot.swerve.getPose2d();
    double xOutput = m_xController.calculate(
      m_currentPose.getX(),
      m_targetPose.getX()
    );
    double yOutput = m_yController.calculate(
      m_currentPose.getY(),
      m_targetPose.getY()
    );
    double rotationOutput =
      m_rotationController.calculate(
        m_currentPose.getRotation().getRadians(),
        m_targetPose.getRotation().getRadians()
      ) +
      DRIVE_TO_POSE.PIGEON_ROTATION_FEEDFORWARD;
    if (
      Utility.isWithinTolerance(
        m_currentPose.getX(),
        m_targetPose.getX(),
        m_xController.getErrorTolerance()
      )
    ) {
      xOutput = 0;
    }
    if (
      Utility.isWithinTolerance(
        m_currentPose.getY(),
        m_targetPose.getY(),
        m_yController.getErrorTolerance()
      )
    ) {
      yOutput = 0;
    }
    if (
      Utility.isWithinTolerance(
        m_currentPose.getRotation().getRadians(),
        m_targetPose.getRotation().getRadians(),
        m_rotationController.getErrorTolerance()
      )
    ) {
      rotationOutput = 0;
    }

    Robot.swerve.driveFieldRelative(xOutput, yOutput, rotationOutput);
    // System.out.println("POSE: " + m_currentPose);
  }

  @Override
  public boolean isFinished() {
    if (
      !Utility.isWithinTolerance(
        m_currentPose.getX(),
        m_targetPose.getX(),
        DRIVE_TO_POSE.WAYPOINT_X_TOLERANCE.in(Meters)
      )
    ) {
      return false;
    }
    if (
      !Utility.isWithinTolerance(
        m_currentPose.getY(),
        m_targetPose.getY(),
        DRIVE_TO_POSE.WAYPOINT_Y_TOLERANCE.in(Meters)
      )
    ) {
      return false;
    }
    if (
      !Utility.isWithinTolerance(
        m_currentPose.getRotation().getRadians(),
        m_targetPose.getRotation().getRadians(),
        DRIVE_TO_POSE.WAYPOINT_ROTATION_TOLERANCE.in(Radians)
      )
    ) {
      return false;
    }
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.stopMotors();
    System.out.println(
      "****\nFINISH DRIVE TO POSE\nPOSE AT FINISH: " + Robot.swerve.getPose2d()
    );
  }
}

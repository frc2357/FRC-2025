package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DRIVE_TO_POSE;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Utility;
import java.util.function.Supplier;

public class DriveToPose extends Command {

  private Supplier<Pose2d> m_targetPoseSupplier;
  private Pose2d m_currentPose;

  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_rotationController;

  private double m_driverX;
  private double m_driverY;
  private double m_driverRoto;

  private double m_xOutput;
  private double m_yOutput;
  private double m_rotoOutput;

  private static final double m_speedAt12VoltsMPS =
    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  public DriveToPose(Supplier<Pose2d> targetPose) {
    addRequirements(Robot.swerve);

    m_targetPoseSupplier = targetPose;
    m_xController = DRIVE_TO_POSE.X_TRANSLATION_PID_CONTROLLER;
    m_yController = DRIVE_TO_POSE.Y_TRANSLATION_PID_CONTROLLER;
    m_rotationController = DRIVE_TO_POSE.ROTATION_PID_CONTROLLER;
  }

  @Override
  public void initialize() {
    m_currentPose = Robot.swerve.getAllianceRelativePose2d();
    m_xController.setTolerance(DRIVE_TO_POSE.X_TOLERANCE.in(Meters));
    m_yController.setTolerance(DRIVE_TO_POSE.Y_TOLERANCE.in(Meters));
    m_rotationController.setTolerance(
      DRIVE_TO_POSE.ROTATION_TOLERANCE.in(Radians)
    );

    m_xController.setSetpoint(m_targetPoseSupplier.get().getX());
    m_yController.setSetpoint(m_targetPoseSupplier.get().getY());
    m_rotationController.setSetpoint(
      m_targetPoseSupplier.get().getRotation().getRadians()
    );

    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

    m_xController.reset();
    m_yController.reset();
    m_rotationController.reset();
    System.out.println(
      "****\nSTART DRIVE TO POSE\nPOSE AT START: " +
      Robot.swerve.getFieldRelativePose2d()
    );
  }

  @Override
  public void execute() {
    updateInformation();
    if (m_driverX != 0 || m_driverY != 0 || m_driverRoto != 0) {
      m_xOutput = m_driverX * m_speedAt12VoltsMPS;
      m_yOutput = m_driverY * m_speedAt12VoltsMPS;
      m_rotoOutput =
        m_driverRoto *
        Constants.SWERVE.MAX_ANGULAR_VELOCITY.in(RotationsPerSecond);
    } else {
      m_xOutput = m_xController.calculate(
        m_currentPose.getX(),
        m_targetPoseSupplier.get().getX()
      );
      if ( // are we in line with our x coordinate goal
        Utility.isWithinTolerance(
          m_currentPose.getX(),
          m_targetPoseSupplier.get().getX(),
          m_xController.getPositionTolerance()
        )
      ) {
        m_xOutput = 0;
      }
      m_yOutput = m_yController.calculate(
        m_currentPose.getY(),
        m_targetPoseSupplier.get().getY()
      );
      if ( // are we in line with our y coordinate goal
        Utility.isWithinTolerance(
          m_currentPose.getY(),
          m_targetPoseSupplier.get().getY(),
          m_yController.getPositionTolerance()
        )
      ) {
        m_yOutput = 0;
      }

      // System.out.println("X = " + m_xOutput);
      // System.out.println("Y = " + m_yOutput);
      m_rotoOutput =
        m_rotationController.calculate(
          m_currentPose.getRotation().getRadians(),
          m_targetPoseSupplier.get().getRotation().getRadians()
        ) +
        DRIVE_TO_POSE.PIGEON_ROTATION_FEEDFORWARD;
      if ( // are we in line with our rotation goal
        Utility.isWithinTolerance(
          m_currentPose.getRotation().getRadians(),
          m_targetPoseSupplier.get().getRotation().getRadians(),
          m_rotationController.getPositionTolerance()
        )
      ) {
        m_rotoOutput = 0;
      }
    }
    // System.out.println("ROTO = " + m_rotoOutput);
    Robot.swerve.driveFieldRelative(m_xOutput, m_yOutput, m_rotoOutput);
  }

  @Override
  public boolean isFinished() {
    var targetPose = m_targetPoseSupplier.get();
    if (
      !Utility.isWithinTolerance(
        m_currentPose.getX(),
        targetPose.getX(),
        DRIVE_TO_POSE.X_TOLERANCE.in(Meters)
      )
    ) {
      return false;
    }
    if (
      !Utility.isWithinTolerance(
        m_currentPose.getY(),
        targetPose.getY(),
        DRIVE_TO_POSE.Y_TOLERANCE.in(Meters)
      )
    ) {
      return false;
    }
    if (
      !Utility.isWithinTolerance(
        m_currentPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians(),
        DRIVE_TO_POSE.ROTATION_TOLERANCE.in(Radians)
      )
    ) {
      return false;
    }
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    m_currentPose = Robot.swerve.getAllianceRelativePose2d();
    System.out.println(
      "DRIVE TO POSE FINISH****\nPOSE AT FINISH: " + m_currentPose
    );
    if (m_driverX == 0 && m_driverY == 0 && m_driverRoto == 0) {
      Robot.swerve.stopMotors();
    }
  }

  private void updateInformation() {
    m_driverX = Utility.deadband(Robot.driverControls.getY(), 0.05);
    m_driverY = Utility.deadband(Robot.driverControls.getX(), 0.05);
    m_driverRoto = Utility.deadband(Robot.driverControls.getRotation(), 0.05);
    m_currentPose = Robot.swerve.getAllianceRelativePose2d();
  }
}

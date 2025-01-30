package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DRIVE_TO_POSE.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.Utility;
import java.util.function.Function;

public class DriveToReef extends Command {

  private final Pose2d[] m_targetPoses;
  private int m_currTargetIndex = 0;

  private Pose2d m_currPose;

  private DriveToPose m_currDriveToPose;

  public DriveToReef(Pose2d... poseSetpoints) {
    m_targetPoses = poseSetpoints;
  }

  @Override
  public void initialize() {
    m_currPose = Robot.swerve.getAllianceRelativePose2d();
    m_currDriveToPose = new DriveToPose(getTargetSupplier()); // make a DriveToPose that we have control of
    m_currDriveToPose.schedule();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    if (!m_currDriveToPose.isScheduled()) {}
    if (isAtTarget(m_targetPoses[m_currTargetIndex], m_currPose)) {
      return m_currTargetIndex++ == m_targetPoses.length;
    }
    return true;
  }

  @Override
  public void end(boolean interrupted) {}

  private boolean isAtTarget(Pose2d targetPose, Pose2d currPose) {
    if (
      !Utility.isWithinTolerance(
        targetPose.getX(),
        currPose.getX(),
        X_TOLERANCE.in(Meters)
      )
    ) {
      return false;
    }
    if (
      !Utility.isWithinTolerance(
        targetPose.getY(),
        currPose.getY(),
        Y_TOLERANCE.in(Meters)
      )
    ) {
      return false;
    }
    if (
      !Utility.isWithinTolerance(
        targetPose.getRotation().getDegrees(),
        currPose.getRotation().getDegrees(),
        ROTATION_TOLERANCE.in(Degrees)
      )
    ) {
      return false;
    }
    return true;
  }

  private boolean isCurrentTargetValid(
    Pose2d currentTargetPose,
    Pose2d lastTargetPose,
    Pose2d currPose
  ) {
    // find which direction we want to go (pos or neg in x/y axes)

    // check if were on the right side of the reef

    // if not, use adjusted setpoints to curve around the reef towards goal, else continue

    // find the distance between current pose and current target

    // find the distance between current pose and last target

    // check which target is closer

    // use the closer target, as long as were not too close to the reef.

    // if too close to reef, make a middle setpoint between current pose and new target, with slight adjustments.

  }

  private Function<Pose2d, Pose2d> getTargetSupplier() {
    return new Function<Pose2d, Pose2d>() {
      @Override
      public Pose2d apply(Pose2d currPose) {}
    };
  }
}

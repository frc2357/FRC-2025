package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DRIVE_TO_POSE.*;
import static frc.robot.Constants.FIELD.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FIELD;
import frc.robot.Robot;
import frc.robot.controls.controllers.ButtonboardController;
import frc.robot.controls.controllers.ButtonboardController.ReefSide;
import frc.robot.util.Utility;
import java.util.function.Function;

public class DriveToReef extends Command {

  private Pose2d m_currPose;

  private Pose2d m_currentTarget;
  private Pose2d m_previousTarget;

  private Pose2d m_finalGoal;

  private DriveToPose m_currDriveToPose;

  public DriveToReef() {}

  @Override
  public void initialize() {
    m_finalGoal = getGoalFromButtonboard();
    // if final goal is null, then somethings up, and we dont have any clue where to go from here.
    if (m_finalGoal == null) {
      this.cancel(); // so we wont go anywhere from here, and just cancel instead.
      return;
    }
    m_currPose = Robot.swerve.getAllianceRelativePose2d();
    m_currDriveToPose = new DriveToPose(getTargetFunction()); // make a DriveToPose that we have control of
    m_currDriveToPose.schedule();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    if (!m_currDriveToPose.isScheduled()) {
      if (!isAtGoal()) {
        m_currDriveToPose = new DriveToPose(getTargetFunction());
        return false;
      }
    }
    return isAtGoal();
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

  private boolean isAtGoal() {
    return isAtTarget(m_finalGoal, m_currPose);
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

  private Pose2d getGoalFromButtonboard() {
    ReefSide goal = Robot.buttonboard.getSelectedReefSide();

    switch (goal) {
      case A:
        return BRANCH_A;
      case B:
        return BRANCH_B;
      case C:
        return BRANCH_C;
      case D:
        return BRANCH_D;
      case E:
        return BRANCH_E;
      case F:
        return BRANCH_F;
      default:
        return null; // if this happens, somethings wrong.
    }
  }

  private Function<Pose2d, Pose2d> getTargetFunction() {
    return new Function<Pose2d, Pose2d>() {
      @Override
      public Pose2d apply(Pose2d currPose) {
        return new Pose2d();
      }
    };
  }
}

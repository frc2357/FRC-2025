package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DRIVE_TO_POSE.BRANCH_GOAL;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;

public class DriveToReef extends DriveToPoseHandler {

  private BRANCH_GOAL m_goal;
  private Rotation2d m_rotationGoal;

  public DriveToReef(
    RouteAroundReef routeAroundReef,
    BRANCH_GOAL goal,
    Rotation2d rotationGoal
  ) {
    this(
      routeAroundReef,
      BRANCH_GOAL.BRANCH_A,
      REEF.BRANCH_A.getRotation(),
      null
    );
  }

  public DriveToReef(
    RouteAroundReef routeAroundReef,
    BRANCH_GOAL goal,
    Rotation2d rotationGoal,
    Command finalApproachCommand
  ) {
    super(routeAroundReef, finalApproachCommand);
    m_goal = goal;
    m_rotationGoal = rotationGoal;
  }

  @Override
  public void initialize() {
    // m_finalGoal = REEF.BRANCH_A;
    Pose2d newGoal = Robot.camManager.getFieldRelativeBranchPose(m_goal);
    if (newGoal == null) {
      return;
    }
    newGoal = new Pose2d(
      Robot.camManager.getFieldRelativeBranchPose(m_goal).getTranslation(),
      m_rotationGoal
    );
    System.out.println("***** START OF INIT *****");
    System.out.println("NEW GOAL   - " + newGoal);
    System.out.println("CURR POSE  - " + m_currPose);
    // m_finalGoal = newGoal;
    System.out.println("FINAL GOAL - " + m_finalGoal);
    System.out.println("***** END OF INIT *******");
    super.initialize();
  }

  @Override
  public Pose2d getNewTarget(Pose2d currTarget, Pose2d currPose) {
    System.out.println("CURR POSE  - " + currPose);
    System.out.println("CURR TAR   - " + currTarget);
    System.out.println("FINAL GOAL - " + m_finalGoal);
    // Pose2d newGoal = Robot.camManager.getFieldRelativeBranchPose(m_goal);
    // if (
    //   newGoal != null &&
    //   !newGoal.equals(Pose2d.kZero) &&
    //   new Transform2d(m_finalGoal, newGoal).getTranslation().getNorm() > 0.35
    // ) m_finalGoal = new Pose2d(newGoal.getTranslation(), m_rotationGoal);
    // m_goalOutputFieldTypePub.set("Field2d");
    // Pose2d pose = newGoal;
    // m_goalOutputFieldPub.accept(
    //   new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() }
    // );
    return super.getNewTarget(currTarget, currPose);
  }
}

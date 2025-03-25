package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DRIVE_TO_POSE.BRANCH_GOAL;
import frc.robot.Constants.FIELD.REEF;
import frc.robot.Robot;

public class DriveToReef extends DriveToPoseHandler {

  private BRANCH_GOAL m_goal;

  public DriveToReef(RouteAroundReef routeAroundReef, BRANCH_GOAL goal) {
    this(routeAroundReef, BRANCH_GOAL.BRANCH_A, null);
  }

  public DriveToReef(
    RouteAroundReef routeAroundReef,
    BRANCH_GOAL goal,
    Command finalApproachCommand
  ) {
    super(routeAroundReef, finalApproachCommand);
    m_goal = goal;
  }

  @Override
  public void initialize() {
    m_finalGoal = Robot.swerve.getFieldRelativePose2d();
    Pose2d newGoal = Robot.camManager.getFieldRelativeBranchPose(m_goal);
    if (newGoal == null) {
      return;
    }
    newGoal = Robot.camManager.getFieldRelativeBranchPose(m_goal);
    System.out.println("***** START OF INIT *****");
    System.out.println("NEW GOAL   - " + newGoal);
    System.out.println("CURR POSE  - " + m_currPose);
    m_finalGoal = newGoal;
    System.out.println("FINAL GOAL - " + m_finalGoal);
    System.out.println("***** END OF INIT *******");
    super.initialize();
  }

  @Override
  public Pose2d getNewTarget(Pose2d currTarget, Pose2d currPose) {
    // Pose2d newGoal = Robot.camManager.getFieldRelativeBranchPose(m_goal);
    // if (
    //   newGoal != null &&
    //   new Transform2d(m_finalGoal, newGoal).getTranslation().getNorm() > 0.25
    // ) {
    //   System.out.println("NEW GOAL = " + newGoal);
    //   m_finalGoal = newGoal;
    // }
    return super.getNewTarget(currTarget, currPose);
  }
}
